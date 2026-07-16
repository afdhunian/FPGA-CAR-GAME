module render_static ( 
    input  wire       clk,
    input  wire       video_on,
	 input  wire 		 reset,
    input  wire [9:0] x,
    input  wire [9:0] y,
    input  wire       steer_left,   // active-high: hold to move the car left
    input  wire       steer_right,  // active-high: hold to move the car right
    input  wire       start_btn,    // active-high: press once to begin from the start screen
    output reg  [7:0] red,
    output reg  [7:0] green,
    output reg  [7:0] blue,
	 output reg [15:0] score,
    output reg        game_over,
    output reg        collision    // 1-cycle pulse (same clk domain) the instant
                                    // the car's box and the obstacle's box overlap.
                                    // Sample it in another always @(posedge clk)
                                    // block for score/lives/game-over -- it will
                                    // only be high for a single clk, not held.
);

    // ------------------------------------------------------------------
    // Start screen: nothing in the world moves/updates until the player
    // presses start_btn for the first time. Same freeze mechanism as
    // game_over (every world-state always block below checks BOTH
    // !game_over and game_started), so this reuses that pattern rather
    // than adding a whole separate code path.
    // ------------------------------------------------------------------
    reg game_started;
    reg start_btn_prev;
    initial game_started   = 1'b0;
    initial start_btn_prev = 1'b0;

    always @(posedge clk) begin
        if (reset) begin
            game_started   <= 1'b0;
            start_btn_prev <= 1'b0;
        end else if (x == 10'd0 && y == 10'd0) begin   // sekali per frame
            start_btn_prev <= start_btn;
            if (!game_started && start_btn && !start_btn_prev)
                game_started <= 1'b1;
        end
    end


    wire [9:0] dy;
    wire [9:0] road_left;
    wire [9:0] road_right;
    wire [9:0] road_width;
    wire [9:0] line_margin;
    wire [9:0] line_thick;

    assign dy = (y >= 10'd120) ? (y - 10'd120) : 10'd0;

    // How fast the world scrolls toward the camera each frame -- used by
    // both the road curve timing below and the dashed-line/tree motion
    // further down. This is the BASE rate; world_scroll_speed (below)
    // adds the same difficulty bonus OBSTACLE_SPEED gets, so the road
    // and trees visibly speed up right along with the obstacles instead
    // of staying at a fixed pace while only the obstacles get faster.
    localparam SCROLL_SPEED = 10'd3;

    // Actual scroll rate applied to seg_dist/scroll_y below.
    // obstacle_speed_bonus is declared further down (next to
    // OBSTACLE_SPEED) but that's fine -- it's a plain wire, so the
    // forward reference resolves normally in Verilog.
    wire [9:0] world_scroll_speed = SCROLL_SPEED + obstacle_speed_bonus;

    // Road converges toward a narrow point at the horizon and widens
    // toward the camera. ROAD_HALF_FAR is the half-width right at the
    // horizon (small = looks "far away"); it grows linearly with dy as
    // y increases, so the road gets progressively wider lower down the
    // screen (closer to the camera).
    localparam ROAD_CENTER_X = 10'd320;
    localparam ROAD_HALF_FAR = 10'd20;

    // ------------------------------------------------------------------
    // Road curve: bend the top of the road sideways while the bottom
    // stays anchored at ROAD_CENTER_X.
    //
    // The sideways bend at any distance is curve_amount * (360 - dy),
    // scaled down by /8 at the very end (see curve_shift_at below) --
    // so it runs from a full 45x factor at the horizon (dy=0) down to
    // 0 at the very bottom of the screen (dy=360, i.e. y=480), same
    // range as before. Keeping dy un-truncated until the final divide
    // (instead of a coarse dy>>3 done up front) is what makes the
    // curve trace one smooth arc per scanline instead of a visible
    // 8-row "staircase" on sharper bends.
    //
    // This is fully automatic and cycles through 4 phases, each lasting
    // SEGMENT_LENGTH pixels of travel: straight -> curve right ->
    // straight -> curve left -> repeat. curve_amount eases toward
    // whatever the current phase's target is by CURVE_STEP per frame,
    // so transitions are smooth instead of an instant kink.
    // ------------------------------------------------------------------
    // CURVE_MAX_CAP was previously limited to 6 because curved_center_x
    // (and the obstacle version of the same math) used to truncate
    // straight to unsigned with no bounds check, which would underflow
    // and glitch/wrap the road near the horizon past that point. Both
    // now go through clamp_center_x (below) before being used, so
    // pushing this much higher is safe -- the road just visually pins
    // at its max bend instead of wrapping.
    localparam signed CURVE_MAX_CAP       = 16;
    localparam signed CURVE_MAX_START     = 3;   // how sharp the very first bend is
    localparam signed CURVE_MAX_INCREMENT = 1;   // how much sharper each new loop gets
    localparam         CURVE_STEP    = 1;      // how much curve_amount moves each time it updates
    localparam         CURVE_UPDATE_DIVIDER = 8'd4; // only step every N frames -- bigger = slower/smoother easing
    localparam         SEGMENT_LENGTH = 20'd800; // pixels of travel per phase (straight or curved)

    localparam PHASE_STRAIGHT_1 = 2'd0;
    localparam PHASE_RIGHT      = 2'd1;
    localparam PHASE_STRAIGHT_2 = 2'd2;
    localparam PHASE_LEFT       = 2'd3;

    reg [19:0]        seg_dist;
    reg [1:0]         phase;         // cycles 0 -> 1 -> 2 -> 3 -> 0 ...
    reg signed [6:0]  curve_amount;  // -current_curve_max .. +current_curve_max
    reg [7:0]         curve_update_counter;
    reg signed [6:0]  current_curve_max;  // grows by CURVE_MAX_INCREMENT each full loop, capped at CURVE_MAX_CAP

    initial current_curve_max = CURVE_MAX_START;

    wire signed [6:0] target_curve = (phase == PHASE_RIGHT) ? current_curve_max :
                                      (phase == PHASE_LEFT)  ? -current_curve_max :
                                                                7'sd0;

    always @(posedge clk) begin
        if (reset) begin
            seg_dist             <= 20'd0;
            phase                <= PHASE_STRAIGHT_1;
            curve_amount         <= 7'sd0;
            curve_update_counter <= 8'd0;
            current_curve_max    <= CURVE_MAX_START;
        end else if (x == 10'd0 && y == 10'd0 && !game_over && game_started) begin   // once per frame, freeze on game over / before start
            if (seg_dist + world_scroll_speed >= SEGMENT_LENGTH) begin
                seg_dist <= (seg_dist + world_scroll_speed) - SEGMENT_LENGTH;

                // Completing PHASE_LEFT means the next phase wraps back
                // to PHASE_STRAIGHT_1 -- i.e. a full loop just finished.
                // Make the next loop's bends a bit sharper, up to the cap.
                if (phase == PHASE_LEFT && current_curve_max < CURVE_MAX_CAP)
                    current_curve_max <= current_curve_max + CURVE_MAX_INCREMENT;

                phase <= phase + 2'd1;      // wraps 0,1,2,3,0,... automatically
            end else begin
                seg_dist <= seg_dist + world_scroll_speed;
            end

            if (curve_update_counter >= CURVE_UPDATE_DIVIDER - 8'd1) begin
                curve_update_counter <= 8'd0;
                if (curve_amount < target_curve)
                    curve_amount <= curve_amount + CURVE_STEP;
                else if (curve_amount > target_curve)
                    curve_amount <= curve_amount - CURVE_STEP;
            end else begin
                curve_update_counter <= curve_update_counter + 8'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Clamps a curved center-x to whatever range keeps a box of the
    // given half-extent fully on screen (0..639). Used for the road
    // itself and for the obstacle, which does the same "bend sideways
    // by curve_amount" math at its own distance -- without this, a
    // strong enough curve_amount pushes the raw signed value negative
    // (or past 639), and truncating that straight to unsigned wraps
    // around instead of clamping, which glitches the road/obstacle
    // right at the horizon. This is what makes it safe to allow a much
    // sharper CURVE_MAX_CAP than before.
    // ------------------------------------------------------------------
    function [9:0] clamp_center_x;
        input signed [13:0] cx;    // widened from [10:0]: curve_shift_at can now return values
                                    // past +-1024 with the quadratic curve below, which would
                                    // silently overflow/wrap an 11-bit signed input before this
                                    // function even got a chance to clamp it
        input [9:0] half_extent;
        reg signed [13:0] lo, hi;
        begin
            lo = $signed({4'd0, half_extent});
            hi = 14'sd639 - $signed({4'd0, half_extent});
            if (cx < lo)
                clamp_center_x = lo[9:0];
            else if (cx > hi)
                clamp_center_x = hi[9:0];
            else
                clamp_center_x = cx[9:0];
        end
    endfunction

    // ------------------------------------------------------------------
    // Shared, full-resolution curve math used by every element that
    // needs "how far sideways is the road bent at this distance" --
    // the road surface itself, both treelines, the car's dynamic road
    // limits, and the obstacle all called this same formula
    // independently before, each with its own coarse `45 - (dy >> 3)`
    // pre-truncation. That truncation only changes once every 8 rows,
    // which is harmless for a single-row check (car/obstacle) but for
    // the road surface -- evaluated on every scanline -- it made the
    // curve edge visibly step in 8px "stairs" instead of tracing one
    // smooth arc, especially on sharper bends. Folding the divide-by-8
    // into the END of the calculation instead keeps full per-row
    // precision the whole way through, so the curve is smooth AND
    // every consumer (road/trees/car/obstacle) stays in perfect
    // agreement with each other.
    // ------------------------------------------------------------------
    function signed [13:0] curve_shift_at;
        input signed [6:0] camt;   // curve_amount (or a snapshot of it)
        input        [9:0] dyv;    // distance from horizon for this row/object
        reg          [9:0] d;           // 360 (horizon) .. 0 (bottom of screen)
        reg          [19:0] d_sq;       // d*d, fits comfortably in 20 bits (max 360*360=129600)
        reg          [9:0] cbase_fine;  // quadratic falloff of d, NOT linear
        reg signed  [17:0] prod;
        begin
            d              = 10'd360 - dyv;
            // Quadratic instead of linear: a plain "cbase_fine = d"
            // makes the road's centerline a straight-line ramp in y --
            // i.e. the road bends like a flat wedge/hinge with sharp,
            // angular corners at the phase transitions instead of
            // tracing an actual arc. Squaring d (then rescaling back
            // down) makes the shift grow slowly near the camera and
            // sharply out near the horizon, like a real road curving
            // away into the distance -- a smooth, rounded bend instead
            // of a perpendicular-looking kink.
            d_sq           = d * d;
            cbase_fine     = d_sq >> 8;
            prod           = camt * $signed({1'b0, cbase_fine});
            curve_shift_at = prod >>> 3;   // same overall /8 scale the old code used
        end
    endfunction

    wire signed [13:0] curve_shift;
    assign curve_shift = curve_shift_at(curve_amount, dy);

    wire signed [13:0] center_x_signed;
    assign center_x_signed = $signed({1'b0, ROAD_CENTER_X}) + curve_shift;

    wire [9:0] curved_center_x;
    assign curved_center_x = clamp_center_x(center_x_signed, ROAD_HALF_FAR + (dy >> 1));

    assign road_left  = curved_center_x - (ROAD_HALF_FAR + (dy >> 1));
    assign road_right = curved_center_x + (ROAD_HALF_FAR + (dy >> 1));

    // Margin scales with the road's current width, so the two edge
    // lines stay proportionally close together near the horizon (where
    // the road is narrow) and spread out naturally as the road widens.
    assign road_width  = road_right - road_left;
    assign line_margin = road_width >> 5;

    // Line thickness also scales with road width -- thin near the
    // horizon, thick near the camera -- instead of a fixed pixel count
    // that would look oversized on the narrow far end of the road.
    // Floored at 2px so it never disappears entirely up near the top.
    assign line_thick = (road_width >> 4 < 10'd2) ? 10'd2 : (road_width >> 6);

    wire road_area;
    wire left_solid_line;
    wire right_solid_line;
    wire center_dash;
    wire car_body;
    wire car_window;
    wire car_tire_left;
    wire car_tire_right;

    // ------------------------------------------------------------------
    // Lightweight anti-aliasing: instead of a hard color cutoff at each
    // boundary, blend the ONE pixel right at the edge using the
    // sub-pixel remainder that our integer math would otherwise just
    // throw away. This turns visible "staircase" jumps -- especially on
    // the white lines, whose margin/thickness only update once every
    // ~16 rows because of the >>4 shift above -- into a soft gradient.
    // ------------------------------------------------------------------
    function [7:0] blend4;
        input [7:0] a, b;   // a = color at weight 0, b = color at weight ~15
        input [3:0] w;      // 0 .. 15
        reg [11:0] sum;
        begin
            sum = (a * (5'd16 - {1'b0, w})) + (b * {1'b0, w});
            blend4 = sum[11:4];   // divide by 16
        end
    endfunction

    // Road/shoulder boundary: (dy >> 1) throws away a 0.5px remainder
    // every other row (dy[0]). That remainder becomes a 50% blend
    // weight for the pixel sitting just outside the integer edge.
    wire [3:0] road_edge_w;
    assign road_edge_w = dy[0] ? 4'd8 : 4'd0;

    wire left_road_edge_aa, right_road_edge_aa;
    assign left_road_edge_aa  = (y >= 10'd120) && (x == road_left  - 10'd1);
    assign right_road_edge_aa = (y >= 10'd120) && (x == road_right + 10'd1);

    // White line outer edge: line_margin only changes once every ~16
    // rows (road_width >> 4). The 4 bits that shift throws away become
    // the blend weight for the asphalt pixel sitting right next to it.
    wire [3:0] line_edge_w;
    assign line_edge_w = road_width[3:0];

    wire left_line_edge_aa, right_line_edge_aa;
    assign left_line_edge_aa  = road_area && (x == road_left  + line_margin - 10'd1);
    assign right_line_edge_aa = road_area && (x == road_right - line_margin + 10'd1);

    // Dirt/gravel shoulder removed -- grass now runs right up to the
    // trotoar/curb instead of a separate dirt strip beyond it.

    // Trotoar (sidewalk curb): a thin black-and-white striped band
    // sitting right at the road's edge, between the asphalt and the
    // grass -- like the painted kerb stones on a real road.
    localparam TROTOAR_WIDTH = 10'd6;
    wire trotoar_left, trotoar_right;

    // A touch of texture on the asphalt itself so it doesn't look like
    // a single flat painted block. Cheap and synthesizable: just XOR a
    // handful of position bits together to get a fixed speckle pattern.
    wire asphalt_fleck;

    // ------------------------------------------------------------------
    // Motion illusion: a counter that advances once per frame and gets
    // added to the y-coordinate of everything that should appear to
    // slide toward the camera (dashed center line, asphalt speckle,
    // trees). The car itself stays fixed on screen -- exactly like a
    // real driving game, the world moves past the car instead of the
    // car moving through the world.
    //
    // SCROLL_SPEED (declared earlier, next to the curve timer) is the base
    // rate; world_scroll_speed adds the same difficulty bonus obstacles
    // get, so the trees/dashes/asphalt appear to speed up right along
    // with the obstacles as the score climbs.
    reg [9:0] scroll_y;

    always @(posedge clk) begin
        if (reset)
            scroll_y <= 10'd0;
        else if (x == 10'd0 && y == 10'd0 && !game_over && game_started)
            scroll_y <= scroll_y + world_scroll_speed;
    end

    // NOTE: subtract, not add. Sampling the pattern at (y - scroll_y)
    // makes a fixed dash/fleck boundary drift to LARGER y as scroll_y
    // grows -- i.e. down the screen, toward the camera -- which reads
    // as forward motion. Adding scroll_y instead makes it drift the
    // other way (up toward the horizon), which looks like reversing.
    wire [9:0] anim_y = y - scroll_y;

    assign asphalt_fleck = x[1] ^ x[4] ^ anim_y[2] ^ anim_y[5];

    // Fine per-pixel color variation for the grass -- same cheap
    // XOR-speckle idiom as asphalt_fleck, just two independent bit
    // combos so there are 4 close, subtle shades instead of one flat
    // block of color. Deliberately fine-grained (low-order bits), not
    // big blocks, so it reads as texture/noise rather than patches.
    wire grass_speck_a, grass_speck_b;
    assign grass_speck_a = x[0] ^ anim_y[0];
    assign grass_speck_b = x[2] ^ anim_y[2];

    // Grass decorations (flowers, small rocks, tall-grass clumps) have
    // all been removed -- replaced by more houses along the roadside
    // instead.
    // Wrap a tree's base y-position (ty) forward by scroll_y, looping
    // back to the horizon once it would scroll past the bottom of the
    // screen. Keeps every tree cycling endlessly down the road instead
    // of just sliding off and disappearing after one pass.
    function [9:0] wrap_ty;
        input [9:0] ty;
        input [9:0] scroll;
        localparam [9:0] Y_MIN  = 10'd120;          // horizon, top of road
        localparam [9:0] Y_MAX  = 10'd480;          // bottom of screen
        localparam [9:0] RANGE  = Y_MAX - Y_MIN;     // 360
        reg [10:0] offset;   // 11 bits: max (ty-Y_MIN)+scroll = 360+1023 = 1383, needs >10 bits
        begin
            // ------------------------------------------------------------
            // BUG FIX: this used to be `(ty - Y_MIN + scroll) % RANGE`.
            // RANGE (360) is NOT a power of 2, so Quartus can't turn that
            // % into a free bit-slice -- it has to synthesize a full
            // division circuit. This function is called 16 times (8 left
            // trees + 8 right trees), so that was 16 separate dividers,
            // which is exactly the kind of thing that makes Analysis &
            // Synthesis balloon in memory/time and crash Quartus on a
            // machine with limited RAM.
            //
            // Since the value being wrapped ((ty - Y_MIN) + scroll) can
            // only ever reach 360 + 1023 = 1383 at most, "mod 360" here
            // is exactly the same as subtracting 360 repeatedly until
            // what's left is under 360 -- at most 4 subtractions ever
            // needed. Each subtraction is just a comparator + subtractor,
            // orders of magnitude cheaper to synthesize than a divider.
            // ------------------------------------------------------------
            offset = {1'b0, ty} - {1'b0, Y_MIN} + {1'b0, scroll};
            if (offset >= {1'b0, RANGE}) offset = offset - {1'b0, RANGE};
            if (offset >= {1'b0, RANGE}) offset = offset - {1'b0, RANGE};
            if (offset >= {1'b0, RANGE}) offset = offset - {1'b0, RANGE};
            if (offset >= {1'b0, RANGE}) offset = offset - {1'b0, RANGE};
            wrap_ty = Y_MIN + offset[9:0];
        end
    endfunction

    // ------------------------------------------------------------------
    // Trees dotted along the grass on both sides of the road. Each tree
    // is a round canopy (circle, via dx*dx+dy*dy <= r*r) sitting on a
    // rectangular trunk. Implemented as a function so adding more trees
    // is just one more localparam set + one more function call, instead
    // of copy-pasting a full geometry block each time.
    //
    // Cel-shaded: a fixed "light from the left" split (px < tx = lit
    // side, else shadow side) on both the canopy and trunk, plus a
    // thin darker outline ring around the canopy -- flat single-color
    // circles/rectangles read as very plain, this gives each tree a bit
    // of shape/volume for almost no extra cost.
    //
    // Returns: 3'b001 = canopy lit, 3'b010 = canopy shadow,
    //          3'b011 = canopy outline, 3'b100 = trunk lit,
    //          3'b101 = trunk shadow, 3'b000 = none
    // ------------------------------------------------------------------
    function [2:0] tree_at;
        input [9:0] px, py;   // pixel under test
        input [9:0] tx, ty;   // tree position: x center, y = ground/base
        input [4:0] cr;       // canopy radius
        input [4:0] tw, th;   // trunk width, trunk height
        reg   [9:0] cy;       // canopy center y
        reg   [9:0] dx, dyv;
        reg   [19:0] dist2, r2, inner_r2;
        reg   [4:0] inner_cr;
        reg         is_lit;
        begin
            cy   = ty - th - cr + 10'd3;  // canopy overlaps top of trunk a bit
            dx   = (px >= tx) ? (px - tx) : (tx - px);
            dyv  = (py >= cy) ? (py - cy) : (cy - py);
            dist2 = dx * dx + dyv * dyv;
            r2    = {15'd0, cr} * {15'd0, cr};

            inner_cr = (cr > 5'd1) ? (cr - 5'd1) : 5'd0;
            inner_r2 = {15'd0, inner_cr} * {15'd0, inner_cr};

            is_lit = (px < tx); // fixed "light from the left" for every tree

            if (dist2 <= r2) begin
                if (dist2 > inner_r2)
                    tree_at = 3'b011; // thin outline ring
                else if (is_lit)
                    tree_at = 3'b001; // canopy lit side
                else
                    tree_at = 3'b010; // canopy shadow side
            end else if ((px >= tx - {5'd0, tw >> 1}) && (px <= tx + {5'd0, tw >> 1}) &&
                     (py >= ty - {5'd0, th}) && (py <= ty)) begin
                tree_at = is_lit ? 3'b100 : 3'b101; // trunk lit/shadow side
            end else
                tree_at = 3'b000;
        end
    endfunction

    // ------------------------------------------------------------------
    // Houses: small procedural buildings (rectangular wall + triangular
    // roof + a door + two windows) scattered further back in the grass,
    // behind the treeline, so the roadside doesn't look empty/sepi.
    // Same function-based approach as tree_at above: one call per
    // instance, cel-shaded with the same fixed "light from the left"
    // split used everywhere else (car, trees, signs).
    //
    // The roof silhouette reuses the exact cross-multiplication trick
    // already used for the background mountains and the sign arrow
    // (dx*height <= half_width*height_from_apex) so no division is
    // needed to test whether a pixel is inside the sloped roof.
    //
    // Returns: 3'b001 = wall lit, 3'b010 = wall shadow, 3'b011 = roof,
    //          3'b100 = window, 3'b101 = door, 3'b000 = none
    // ------------------------------------------------------------------
    function [2:0] house_at;
        input [9:0] px, py;    // pixel under test
        input [9:0] hx, hy;    // house position: x center, y = ground/base
        input [4:0] hw;        // wall half-width
        input [4:0] hh;        // wall height
        input [4:0] rh;        // roof height (rise)
        input [4:0] rhw;       // roof half-width (overhang, >= hw)
        reg   [9:0] roof_base_y, apex_y;
        reg   [9:0] dx, dy_from_apex;
        reg         is_lit;
        reg   [9:0] door_hw, door_top;
        reg   [9:0] win_off, win_half, win_top, win_bot;
        begin
            roof_base_y = hy - {5'd0, hh};
            apex_y      = roof_base_y - {5'd0, rh};
            dx          = (px >= hx) ? (px - hx) : (hx - px);
            is_lit      = (px < hx);

            door_hw  = {6'd0, hw} >> 2;
            if (door_hw < 10'd2) door_hw = 10'd2;
            door_top = hy - ({6'd0, hh} >> 1);

            win_half = {6'd0, hw} >> 3;
            if (win_half < 10'd1) win_half = 10'd1;
            win_off  = {6'd0, hw} >> 1;
            win_top  = roof_base_y + ({6'd0, hh} >> 3) + 10'd1;
            win_bot  = win_top + (win_half << 1);

            if ((py >= apex_y) && (py < roof_base_y) && (dx <= {5'd0, rhw})) begin
                dy_from_apex = py - apex_y;
                if ((dx * {5'd0, rh}) <= ({5'd0, rhw} * dy_from_apex))
                    house_at = 3'b011; // roof
                else
                    house_at = 3'b000;
            end else if ((py >= roof_base_y) && (py <= hy) && (dx <= {5'd0, hw})) begin
                if ((dx <= door_hw) && (py >= door_top))
                    house_at = 3'b101; // door
                else if ((py >= win_top) && (py <= win_bot) &&
                         (((dx >= win_off - win_half - win_half) && (dx <= win_off - win_half)) ||
                          ((dx >= win_half) && (dx <= win_half + win_half))))
                    house_at = 3'b100; // window (one pane each side of center)
                else
                    house_at = is_lit ? 3'b001 : 3'b010; // wall lit/shadow
            end else
                house_at = 3'b000;
        end
    endfunction

    // ------------------------------------------------------------------
    // Treeline: instead of a handful of trees at fixed spots, generate
    // NUM_TREES_SIDE evenly-spaced trees per side that each cycle
    // through the FULL horizon-to-camera range (via wrap_ty), scale up
    // as they approach (same growth idea as the obstacle), and hug
    // whichever way the road is currently curving -- so the treeline
    // reads as one continuous row from one end of the road to the
    // other, on both sides, at every point in time.
    // ------------------------------------------------------------------
    localparam NUM_TREES_SIDE      = 5; // reduced from 8 -- see note below about compile stability
    localparam [9:0] TREE_Y_MIN    = 10'd120;
    localparam [9:0] TREE_Y_MAX    = 10'd480;
    localparam [9:0] TREE_Y_STEP   = (TREE_Y_MAX - TREE_Y_MIN) / NUM_TREES_SIDE; // 45
    localparam [9:0] RIGHT_PHASE_OFFSET = TREE_Y_STEP >> 1; // stagger right side vs left -- less "mirrored" look
    localparam [9:0] TREE_GAP       = 10'd6;   // gap between road edge and nearest tree edge
    localparam [9:0] TREE_BASE_HALF = 10'd4;   // canopy radius at the horizon (small/far away)
    localparam       TREE_SCALE_SHIFT = 4;     // how fast trees grow with distance

    wire [2:0] tp_left  [0:NUM_TREES_SIDE-1];
    wire [2:0] tp_right [0:NUM_TREES_SIDE-1];

    genvar gi;
    generate
        for (gi = 0; gi < NUM_TREES_SIDE; gi = gi + 1) begin : gen_left_tree
            localparam [9:0] base_ty = TREE_Y_MIN + (TREE_Y_STEP >> 1) + gi * TREE_Y_STEP;

            wire [9:0] this_ty    = wrap_ty(base_ty, scroll_y);
            wire [9:0] this_dy    = this_ty - TREE_Y_MIN;
            wire [9:0] this_half  = TREE_BASE_HALF + (this_dy >> TREE_SCALE_SHIFT);
            wire [9:0] this_rhalf = ROAD_HALF_FAR + (this_dy >> 1);

            wire signed [13:0] this_cshift  = curve_shift_at(curve_amount, this_dy);
            wire signed [13:0] this_censig  = $signed({1'b0, ROAD_CENTER_X}) + this_cshift;
            wire [9:0] this_center = clamp_center_x(this_censig, 10'd5); // just prevents bit-overflow wrap, trees can still sit near the edge

            wire [9:0] this_tx = this_center - this_rhalf - TREE_GAP - this_half;

            assign tp_left[gi] = tree_at(x, y, this_tx, this_ty,
                                          this_half[4:0], this_half[4:0] >> 1, this_half[4:0]);
        end

        for (gi = 0; gi < NUM_TREES_SIDE; gi = gi + 1) begin : gen_right_tree
            localparam [9:0] base_ty = TREE_Y_MIN + (TREE_Y_STEP >> 1) + RIGHT_PHASE_OFFSET + gi * TREE_Y_STEP;

            wire [9:0] this_ty    = wrap_ty(base_ty, scroll_y);
            wire [9:0] this_dy    = this_ty - TREE_Y_MIN;
            wire [9:0] this_half  = TREE_BASE_HALF + (this_dy >> TREE_SCALE_SHIFT);
            wire [9:0] this_rhalf = ROAD_HALF_FAR + (this_dy >> 1);

            wire signed [13:0] this_cshift  = curve_shift_at(curve_amount, this_dy);
            wire signed [13:0] this_censig  = $signed({1'b0, ROAD_CENTER_X}) + this_cshift;
            wire [9:0] this_center = clamp_center_x(this_censig, 10'd5); // just prevents bit-overflow wrap, trees can still sit near the edge

            wire [9:0] this_tx = this_center + this_rhalf + TREE_GAP + this_half;

            assign tp_right[gi] = tree_at(x, y, this_tx, this_ty,
                                           this_half[4:0], this_half[4:0] >> 1, this_half[4:0]);
        end
    endgenerate

    // Reduce all 16 tp values down to the same 5 category flags the
    // color mux already expects -- a synthesizable for-loop, unrolled
    // at compile time since NUM_TREES_SIDE is constant.
    reg tree_canopy_lit, tree_canopy_shadow, tree_canopy_outline, tree_trunk_lit, tree_trunk_shadow;
    integer ti;
    always @(*) begin
        tree_canopy_lit     = 1'b0;
        tree_canopy_shadow  = 1'b0;
        tree_canopy_outline = 1'b0;
        tree_trunk_lit      = 1'b0;
        tree_trunk_shadow   = 1'b0;
        for (ti = 0; ti < NUM_TREES_SIDE; ti = ti + 1) begin
            if (tp_left[ti] == 3'b001 || tp_right[ti] == 3'b001) tree_canopy_lit     = 1'b1;
            if (tp_left[ti] == 3'b010 || tp_right[ti] == 3'b010) tree_canopy_shadow  = 1'b1;
            if (tp_left[ti] == 3'b011 || tp_right[ti] == 3'b011) tree_canopy_outline = 1'b1;
            if (tp_left[ti] == 3'b100 || tp_right[ti] == 3'b100) tree_trunk_lit      = 1'b1;
            if (tp_left[ti] == 3'b101 || tp_right[ti] == 3'b101) tree_trunk_shadow   = 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // Ponds ("kolam"): a couple of irregular water blobs scattered WAY
    // out past the treeline/house/windmill band on each side -- never
    // round, never gridded with the road, trees, houses or windmill.
    // Each pond is the UNION of two overlapping flattened blobs (same
    // "dx^2 + 2*dy^2 <= 2*r^2" fast-ellipse trick the old boulder used,
    // just applied twice at an offset) so the outline reads as a
    // free-form puddle/pond shape instead of a circle. Its along-road
    // row is picked by hashing the slot index across the WHOLE
    // TREE_Y_MIN..TREE_Y_MAX range (an elaboration-time %, free since
    // gi/SEED are compile-time constants -- no runtime divider), so
    // ponds land at essentially arbitrary rows instead of an evenly
    // spaced line, and POND_EXTRA_GAP pushes them well behind the
    // house/windmill footprint so they don't crowd the road, the
    // treeline, or the village band. Same wrap_ty/curve_shift_at
    // perspective math as everything else, so they still scale and
    // bend with the road as they scroll.
    // ------------------------------------------------------------------
    localparam NUM_PONDS_SIDE   = 2;
    localparam [9:0] POND_EXTRA_GAP  = 10'd55; // clearance beyond the whole scenery band's own footprint
    localparam [9:0] POND_BASE_R     = 10'd8;  // sizeable body of water, bigger than a boulder was --
                                                 // kept low enough that BASE_R + max(dy>>SCALE_SHIFT) still
                                                 // fits the 5-bit radius port pond_at takes (no wrap-around
                                                 // at close range)
    localparam       POND_SCALE_SHIFT = TREE_SCALE_SHIFT; // grows with distance like everything else

    // ------------------------------------------------------------------
    // Shape test: union of two flattened blobs (lobe 1 anchored at
    // cx,cy; lobe 2 offset by dx2,dy2 with its own smaller radius r2)
    // so the combined silhouette is an irregular kidney/puddle outline
    // instead of a plain circle. Each lobe uses the same widened-product
    // idiom as the old boulder_at (adx/ady widened before squaring, so
    // far-away pixels can't silently wrap the multiply). A thin "shore"
    // ring (outer-but-not-inner on either lobe) frames the water same
    // as the outline ring did for trees/boulders.
    //
    // Returns: 3'b001 = water lit, 3'b010 = water shadow,
    //          3'b011 = shore/mud rim, 3'b000 = none
    // ------------------------------------------------------------------
    function [2:0] pond_at;
        input [9:0] px, py;
        input [9:0] cx, cy;          // lobe 1 center
        input [4:0] r1;              // lobe 1 vertical half-extent
        input signed [8:0] dx2, dy2; // lobe 2 offset from lobe 1 center
        input [4:0] r2;              // lobe 2 vertical half-extent
        reg signed [10:0] ddx1, ddy1, ddx2, ddy2;
        reg   [10:0] adx1, ady1, adx2, ady2;
        reg signed [10:0] cx2, cy2;
        reg   [4:0]  shrink1, shrink2, inner_r1, inner_r2;
        reg   [21:0] dist2_1, outer1, inner1, dist2_2, outer2, inner2;
        reg          is_lit, in1, in1i, in2, in2i;
        begin
            cx2 = $signed({1'b0, cx}) + dx2;
            cy2 = $signed({1'b0, cy}) + dy2;

            ddx1 = $signed({1'b0, px}) - $signed({1'b0, cx});
            ddy1 = $signed({1'b0, py}) - $signed({1'b0, cy});
            adx1 = ddx1[10] ? (-ddx1) : ddx1;
            ady1 = ddy1[10] ? (-ddy1) : ddy1;

            ddx2 = $signed({1'b0, px}) - cx2;
            ddy2 = $signed({1'b0, py}) - cy2;
            adx2 = ddx2[10] ? (-ddx2) : ddx2;
            ady2 = ddy2[10] ? (-ddy2) : ddy2;

            is_lit = (px < cx);

            shrink1 = (r1 >> 3); if (shrink1 < 5'd1) shrink1 = 5'd1;
            inner_r1 = (r1 > shrink1) ? (r1 - shrink1) : 5'd0;
            shrink2 = (r2 >> 3); if (shrink2 < 5'd1) shrink2 = 5'd1;
            inner_r2 = (r2 > shrink2) ? (r2 - shrink2) : 5'd0;

            dist2_1 = ({11'd0, adx1} * {11'd0, adx1}) + (({11'd0, ady1} * {11'd0, ady1}) <<< 1);
            outer1  = ({11'd0, r1}       * {11'd0, r1})       <<< 1;
            inner1  = ({11'd0, inner_r1} * {11'd0, inner_r1}) <<< 1;

            dist2_2 = ({11'd0, adx2} * {11'd0, adx2}) + (({11'd0, ady2} * {11'd0, ady2}) <<< 1);
            outer2  = ({11'd0, r2}       * {11'd0, r2})       <<< 1;
            inner2  = ({11'd0, inner_r2} * {11'd0, inner_r2}) <<< 1;

            in1  = dist2_1 <= outer1;
            in1i = dist2_1 <= inner1;
            in2  = dist2_2 <= outer2;
            in2i = dist2_2 <= inner2;

            if (in1i || in2i)
                pond_at = is_lit ? 3'b001 : 3'b010; // water lit/shadow
            else if (in1 || in2)
                pond_at = 3'b011; // shore rim
            else
                pond_at = 3'b000;
        end
    endfunction

    wire [2:0] pnd_left  [0:NUM_PONDS_SIDE-1];
    wire [2:0] pnd_right [0:NUM_PONDS_SIDE-1];

    generate
        for (gi = 0; gi < NUM_PONDS_SIDE; gi = gi + 1) begin : gen_left_pond
            localparam [15:0] SEED     = (gi * 16'd28657) + 16'd9973;
            localparam [9:0]  Y_JIT    = SEED % (TREE_Y_MAX - TREE_Y_MIN); // scatter across the WHOLE depth range, not a fixed grid row
            localparam [9:0]  GAP_JIT  = {6'd0, SEED[9:6]} << 2;          // 0..60 px extra depth jitter
            localparam signed LOBE_SX  = SEED[10] ? -1 : 1;
            localparam signed LOBE_SY  = SEED[11] ? -1 : 1;
            localparam [9:0]  base_py  = TREE_Y_MIN + Y_JIT;

            wire [9:0] this_py    = wrap_ty(base_py, scroll_y);
            wire [9:0] this_dy    = this_py - TREE_Y_MIN;
            wire [9:0] this_r     = POND_BASE_R + (this_dy >> POND_SCALE_SHIFT);
            wire [9:0] this_r2    = (this_r >> 1) + (this_r >> 2); // smaller second lobe, ~0.75x
            wire [9:0] this_rhalf = ROAD_HALF_FAR + (this_dy >> 1);
            wire [9:0] this_reach = this_r + (this_r >> 1); // flattened blob reaches ~1.5x its r horizontally

            wire signed [8:0] this_dx2 = LOBE_SX * $signed({3'd0, this_r[5:0]});
            wire signed [8:0] this_dy2 = LOBE_SY * $signed({3'd0, this_r2[5:0]});

            wire signed [13:0] this_cshift = curve_shift_at(curve_amount, this_dy);
            wire signed [13:0] this_censig = $signed({1'b0, ROAD_CENTER_X}) + this_cshift;
            wire [9:0] this_center = clamp_center_x(this_censig, 10'd5);

            wire [9:0] this_px = this_center - this_rhalf - TREE_GAP - SCEN_BASE_GAP - POND_EXTRA_GAP - GAP_JIT - this_reach;

            assign pnd_left[gi] = pond_at(x, y, this_px, this_py, this_r[4:0], this_dx2, this_dy2, this_r2[4:0]);
        end

        for (gi = 0; gi < NUM_PONDS_SIDE; gi = gi + 1) begin : gen_right_pond
            localparam [15:0] SEED     = (gi * 16'd28657) + 16'd41221;
            localparam [9:0]  Y_JIT    = SEED % (TREE_Y_MAX - TREE_Y_MIN);
            localparam [9:0]  GAP_JIT  = {6'd0, SEED[9:6]} << 2;
            localparam signed LOBE_SX  = SEED[10] ? -1 : 1;
            localparam signed LOBE_SY  = SEED[11] ? -1 : 1;
            localparam [9:0]  base_py  = TREE_Y_MIN + Y_JIT;

            wire [9:0] this_py    = wrap_ty(base_py, scroll_y);
            wire [9:0] this_dy    = this_py - TREE_Y_MIN;
            wire [9:0] this_r     = POND_BASE_R + (this_dy >> POND_SCALE_SHIFT);
            wire [9:0] this_r2    = (this_r >> 1) + (this_r >> 2);
            wire [9:0] this_rhalf = ROAD_HALF_FAR + (this_dy >> 1);
            wire [9:0] this_reach = this_r + (this_r >> 1);

            wire signed [8:0] this_dx2 = LOBE_SX * $signed({3'd0, this_r[5:0]});
            wire signed [8:0] this_dy2 = LOBE_SY * $signed({3'd0, this_r2[5:0]});

            wire signed [13:0] this_cshift = curve_shift_at(curve_amount, this_dy);
            wire signed [13:0] this_censig = $signed({1'b0, ROAD_CENTER_X}) + this_cshift;
            wire [9:0] this_center = clamp_center_x(this_censig, 10'd5);

            wire [9:0] this_px = this_center + this_rhalf + TREE_GAP + SCEN_BASE_GAP + POND_EXTRA_GAP + GAP_JIT + this_reach;

            assign pnd_right[gi] = pond_at(x, y, this_px, this_py, this_r[4:0], this_dx2, this_dy2, this_r2[4:0]);
        end
    endgenerate

    // Same reduction idiom as the treeline above.
    reg pond_water_lit, pond_water_shadow, pond_shore;
    integer ri;
    always @(*) begin
        pond_water_lit    = 1'b0;
        pond_water_shadow = 1'b0;
        pond_shore        = 1'b0;
        for (ri = 0; ri < NUM_PONDS_SIDE; ri = ri + 1) begin
            if (pnd_left[ri] == 3'b001 || pnd_right[ri] == 3'b001) pond_water_lit    = 1'b1;
            if (pnd_left[ri] == 3'b010 || pnd_right[ri] == 3'b010) pond_water_shadow = 1'b1;
            if (pnd_left[ri] == 3'b011 || pnd_right[ri] == 3'b011) pond_shore        = 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // Pond shimmer: the water was previously a flat lit/shadow blob with
    // no motion at all. This adds a handful of bright glints that drift
    // slowly across the surface -- a diagonal hash of (x,y) offset by a
    // slowly-incrementing phase, so the pattern itself slides sideways
    // over time instead of sitting still. Cheap: just adds and a mask,
    // same idiom as asphalt_fleck/grass_speck elsewhere in this file.
    // ------------------------------------------------------------------
    localparam [7:0] SHIMMER_PERIOD_DIV = 8'd10; // frames between each shimmer step -- bigger = slower drift

    reg [7:0] shimmer_phase;
    reg [7:0] shimmer_counter;
    initial shimmer_phase   = 8'd0;
    initial shimmer_counter = 8'd0;

    always @(posedge clk) begin
        if (reset) begin
            shimmer_phase   <= 8'd0;
            shimmer_counter <= 8'd0;
        end else if (x == 10'd0 && y == 10'd0 && !game_over && game_started) begin
            if (shimmer_counter >= SHIMMER_PERIOD_DIV - 8'd1) begin
                shimmer_counter <= 8'd0;
                shimmer_phase   <= shimmer_phase + 8'd1;
            end else begin
                shimmer_counter <= shimmer_counter + 8'd1;
            end
        end
    end

    wire [4:0] shimmer_val  = (x[2:0] + y[2:0] + shimmer_phase[4:0]) & 5'h1F;
    wire       pond_shimmer = pond_water_lit && (shimmer_val < 5'd3);

    // ------------------------------------------------------------------
    // A person standing in their yard near the house (head, torso, arms,
    // legs) -- was a scarecrow figure before; same slot/positioning
    // system below is reused as-is (constants keep their SCARECROW_*
    // names for that reason), only the shape+colors actually drawn
    // have changed.
    //
    // NOTE: this used to run on the exact same step cadence as the
    // house/windmill village band (both stepping by 40px, both starting
    // from the same base phase) -- so an instance here and a scenery
    // instance were very often sitting at almost the same depth AND the
    // same side, i.e. permanently locked together ("numpuk"). Fixed by
    // giving this its own step size (based on its own instance count,
    // same idiom every other scenery system here already uses) and a
    // non-half phase offset -- 180 and 40 don't share a small common
    // period, so the two systems now continuously drift in and out of
    // phase with each other instead of staying glued to the same spot.
    // ------------------------------------------------------------------
    localparam NUM_SCARECROWS_SIDE = 2;
    localparam [9:0] SCARECROW_BASE_H  = 10'd10; // overall height at the horizon
    localparam       SCARECROW_SCALE_SHIFT = 4;
    // Self-contained step/gap constants (deliberately NOT reusing
    // SCEN_Y_STEP/SCEN_BASE_GAP -- those belong to the house/windmill
    // village band declared further down in this file, so referencing
    // them here would be a forward reference to a localparam that
    // doesn't exist yet at this point).
    localparam [9:0] SCARECROW_Y_STEP    = (TREE_Y_MAX - TREE_Y_MIN) / NUM_SCARECROWS_SIDE; // own cadence, not the village band's
    localparam [9:0] SCARECROW_PHASE_OFF = (SCARECROW_Y_STEP >> 2) + 10'd15; // decorrelating nudge, deliberately not a clean half-step
    localparam [9:0] SCARECROW_BASE_GAP  = 10'd22; // baseline extra gap beyond the treeline
    localparam [9:0] SCARECROW_RIGHT_OFF = SCARECROW_Y_STEP >> 1; // stagger left vs right side, same idiom as everything else

    // ------------------------------------------------------------------
    // Shape test: a real standing person -- head (with a small hair
    // cap), torso, two arms hanging at the sides, two legs. Same
    // signed-difference/abs idiom used throughout this file for every
    // other silhouette test. The head uses a proper circular distance
    // test (not the cheap Manhattan-diamond some other shapes use)
    // since it's small enough that a diamond would read as an obvious
    // non-circle at this scale.
    //
    // Note: like every other _at function here, this runs across the
    // WHOLE screen every pixel, so the head's distance-squared compare
    // widens its multiply past the natural operand width -- otherwise
    // it silently wraps for pixels far from the person, the same class
    // of bug that caused the bonus star's stray-pixel glitch fixed
    // earlier.
    //
    // Returns: 0 = none, 1 = legs, 2 = torso, 3 = arms, 4 = head (skin), 5 = hair
    // ------------------------------------------------------------------
    function [2:0] person_at;
        input [9:0] px, py;
        input [9:0] cx, cy;  // cx = center x, cy = ground y (feet)
        input [5:0] h;        // overall height
        reg  [9:0] leg_h, torso_h, head_r;
        reg  [9:0] leg_w, leg_gap, torso_hw, arm_w, arm_gap, arm_len;
        reg  [9:0] leg_top, torso_top, head_cy;
        reg signed [10:0] ddx, ddy;
        reg  [10:0] adx, ady;
        reg  [21:0] dist2, r2;
        begin
            head_r  = {4'd0, (h >> 2)}; if (head_r < 10'd3) head_r = 10'd3;
            leg_h   = {4'd0, (h >> 1)};
            torso_h = ({4'd0, h} > (leg_h + (head_r << 1))) ? ({4'd0, h} - leg_h - (head_r << 1)) : 10'd2;

            leg_w    = {5'd0, (h >> 4)}; if (leg_w < 10'd1) leg_w = 10'd1;
            leg_gap  = 10'd1; // small gap between the two legs
            torso_hw = (leg_w <<< 1) + leg_gap; // as wide as both legs plus the gap between them
            arm_w    = leg_w;
            arm_gap  = 10'd1; // small gap between torso side and arm
            arm_len  = torso_h - (torso_h >> 3);

            leg_top   = cy - leg_h;
            torso_top = leg_top - torso_h;
            head_cy   = torso_top - head_r;

            ddx = $signed({1'b0, px}) - $signed({1'b0, cx});
            ddy = $signed({1'b0, py}) - $signed({1'b0, head_cy});
            adx = ddx[10] ? (-ddx) : ddx;
            ady = ddy[10] ? (-ddy) : ddy;

            dist2 = ({11'd0, adx} * {11'd0, adx}) + ({11'd0, ady} * {11'd0, ady});
            r2    = {11'd0, head_r} * {11'd0, head_r};

            if ((py >= leg_top) && (py <= cy) &&
                (((px >= cx - leg_gap - leg_w) && (px <= cx - leg_gap)) ||
                 ((px >= cx + leg_gap)         && (px <= cx + leg_gap + leg_w))))
                person_at = 3'd1; // legs
            else if ((py >= torso_top) && (py <= leg_top) && (px >= cx - torso_hw) && (px <= cx + torso_hw))
                person_at = 3'd2; // torso
            else if ((py >= torso_top) && (py <= torso_top + arm_len) &&
                     (((px >= cx - torso_hw - arm_gap - arm_w) && (px <= cx - torso_hw - arm_gap)) ||
                      ((px >= cx + torso_hw + arm_gap)         && (px <= cx + torso_hw + arm_gap + arm_w))))
                person_at = 3'd3; // arms
            else if (dist2 <= r2)
                person_at = (ddy <= -({1'b0, head_r} >>> 1)) ? 3'd5 : 3'd4; // hair (upper part) vs skin/face
            else
                person_at = 3'd0;
        end
    endfunction

    wire [2:0] scr_left  [0:NUM_SCARECROWS_SIDE-1];
    wire [2:0] scr_right [0:NUM_SCARECROWS_SIDE-1];

    generate
        for (gi = 0; gi < NUM_SCARECROWS_SIDE; gi = gi + 1) begin : gen_left_scarecrow
            localparam [15:0] SEED    = (gi * 16'd19661) + 16'd5081;
            localparam [9:0]  PHASE_JIT = {6'd0, SEED[6:3]};
            localparam [9:0]  GAP_JIT = {6'd0, SEED[13:10]} << 1;
            localparam [9:0]  base_py = TREE_Y_MIN + (SCARECROW_Y_STEP >> 1) + SCARECROW_PHASE_OFF + gi * SCARECROW_Y_STEP + PHASE_JIT;

            wire [9:0] this_py    = wrap_ty(base_py, scroll_y);
            wire [9:0] this_dy    = this_py - TREE_Y_MIN;
            wire [9:0] this_rhalf = ROAD_HALF_FAR + (this_dy >> 1);
            wire [9:0] this_h     = SCARECROW_BASE_H + (this_dy >> SCARECROW_SCALE_SHIFT);

            wire signed [13:0] this_cshift = curve_shift_at(curve_amount, this_dy);
            wire signed [13:0] this_censig = $signed({1'b0, ROAD_CENTER_X}) + this_cshift;
            wire [9:0] this_center = clamp_center_x(this_censig, 10'd5);

            wire [9:0] this_px = this_center - this_rhalf - TREE_GAP - SCARECROW_BASE_GAP - GAP_JIT - (this_h >> 1);

            assign scr_left[gi] = person_at(x, y, this_px, this_py, this_h[5:0]);
        end

        for (gi = 0; gi < NUM_SCARECROWS_SIDE; gi = gi + 1) begin : gen_right_scarecrow
            localparam [15:0] SEED    = (gi * 16'd19661) + 16'd37423;
            localparam [9:0]  PHASE_JIT = {6'd0, SEED[6:3]};
            localparam [9:0]  GAP_JIT = {6'd0, SEED[13:10]} << 1;
            localparam [9:0]  base_py = TREE_Y_MIN + (SCARECROW_Y_STEP >> 1) + SCARECROW_PHASE_OFF + SCARECROW_RIGHT_OFF + gi * SCARECROW_Y_STEP + PHASE_JIT;

            wire [9:0] this_py    = wrap_ty(base_py, scroll_y);
            wire [9:0] this_dy    = this_py - TREE_Y_MIN;
            wire [9:0] this_rhalf = ROAD_HALF_FAR + (this_dy >> 1);
            wire [9:0] this_h     = SCARECROW_BASE_H + (this_dy >> SCARECROW_SCALE_SHIFT);

            wire signed [13:0] this_cshift = curve_shift_at(curve_amount, this_dy);
            wire signed [13:0] this_censig = $signed({1'b0, ROAD_CENTER_X}) + this_cshift;
            wire [9:0] this_center = clamp_center_x(this_censig, 10'd5);

            wire [9:0] this_px = this_center + this_rhalf + TREE_GAP + SCARECROW_BASE_GAP + GAP_JIT + (this_h >> 1);

            assign scr_right[gi] = person_at(x, y, this_px, this_py, this_h[5:0]);
        end
    endgenerate

    // Same reduction idiom as everything else above.
    reg person_legs, person_torso, person_arms, person_head, person_hair;
    integer pi;
    always @(*) begin
        person_legs  = 1'b0;
        person_torso = 1'b0;
        person_arms  = 1'b0;
        person_head  = 1'b0;
        person_hair  = 1'b0;
        for (pi = 0; pi < NUM_SCARECROWS_SIDE; pi = pi + 1) begin
            if (scr_left[pi] == 3'd1 || scr_right[pi] == 3'd1) person_legs  = 1'b1;
            if (scr_left[pi] == 3'd2 || scr_right[pi] == 3'd2) person_torso = 1'b1;
            if (scr_left[pi] == 3'd3 || scr_right[pi] == 3'd3) person_arms  = 1'b1;
            if (scr_left[pi] == 3'd4 || scr_right[pi] == 3'd4) person_head  = 1'b1;
            if (scr_left[pi] == 3'd5 || scr_right[pi] == 3'd5) person_hair  = 1'b1;
        end
    end


    // ------------------------------------------------------------------
    // Windmill: a tall thin tower with 4 blades spinning around a hub
    // at the top. The blades toggle between a "+" and an "x" orientation
    // (windmill_spin, stepped slowly below) -- a cheap two-frame flip
    // animation that reads as rotation without needing any real angle
    // math. Same fixed "light from the left" cel-shading on the tower
    // as every other silhouette in the scene.
    //
    // Returns: 3'b001 = tower lit, 3'b010 = tower shadow,
    //          3'b011 = blade, 3'b100 = hub, 3'b000 = none
    // ------------------------------------------------------------------
    function [2:0] windmill_at;
        input [9:0] px, py;
        input [9:0] wx, wy;   // wx = tower center x, wy = ground/base y
        input [4:0] tw;       // tower half-width
        input [4:0] th;       // tower height
        input [4:0] bl;       // blade half-length (spike length)
        input       spin;     // 0 = "+" orientation, 1 = "x" orientation
        reg   [9:0] hub_y;
        reg signed [10:0] ddx, ddy, sum_s, diff_s;
        reg   [10:0] adx, ady, asum, adiff;
        reg   [4:0]  thick;
        reg          is_lit, plus_blade, x_blade, hub_pixel;
        begin
            hub_y  = wy - {5'd0, th};
            is_lit = (px < wx);
            thick  = (bl >> 2);
            if (thick < 5'd1) thick = 5'd1;

            if ((px >= wx - {5'd0, tw}) && (px <= wx + {5'd0, tw}) && (py >= hub_y) && (py <= wy)) begin
                windmill_at = is_lit ? 3'b001 : 3'b010; // tower lit/shadow
            end else begin
                ddx = $signed({1'b0, px}) - $signed({1'b0, wx});
                ddy = $signed({1'b0, py}) - $signed({1'b0, hub_y});
                adx = ddx[10] ? (-ddx) : ddx;
                ady = ddy[10] ? (-ddy) : ddy;

                sum_s  = ddx + ddy;
                diff_s = ddx - ddy;
                asum   = sum_s[10]  ? (-sum_s)  : sum_s;
                adiff  = diff_s[10] ? (-diff_s) : diff_s;

                plus_blade = ((adx <= {6'd0, thick}) && (ady <= {6'd0, bl})) ||
                             ((ady <= {6'd0, thick}) && (adx <= {6'd0, bl}));
                x_blade    = ((adiff <= ({6'd0, thick} <<< 1)) && (adx <= {6'd0, bl}) && (ady <= {6'd0, bl})) ||
                             ((asum  <= ({6'd0, thick} <<< 1)) && (adx <= {6'd0, bl}) && (ady <= {6'd0, bl}));
                hub_pixel  = (adx <= {6'd0, thick}) && (ady <= {6'd0, thick});

                if (hub_pixel)
                    windmill_at = 3'b100;
                else if (spin ? x_blade : plus_blade)
                    windmill_at = 3'b011;
                else
                    windmill_at = 3'b000;
            end
        end
    endfunction

    // Slow "does it spin now" toggle -- steps once every WINDMILL_SPIN_
    // PERIOD frames, same "step counter" idiom used everywhere else in
    // this file, so the blade flip is a fixed cadence independent of
    // the difficulty-linked world_scroll_speed.
    localparam [7:0] WINDMILL_SPIN_PERIOD = 8'd24;
    reg windmill_spin;
    reg [7:0] windmill_spin_counter;
    initial windmill_spin         = 1'b0;
    initial windmill_spin_counter = 8'd0;

    always @(posedge clk) begin
        if (reset) begin
            windmill_spin         <= 1'b0;
            windmill_spin_counter <= 8'd0;
        end else if (x == 10'd0 && y == 10'd0 && !game_over && game_started) begin
            if (windmill_spin_counter >= WINDMILL_SPIN_PERIOD - 8'd1) begin
                windmill_spin_counter <= 8'd0;
                windmill_spin         <= ~windmill_spin;
            end else begin
                windmill_spin_counter <= windmill_spin_counter + 8'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Village scenery: houses AND windmills scattered along both sides
    // of the road, further back than the treeline. Unlike the evenly-
    // spaced treeline, each of the NUM_SCENERY_SIDE slots per side picks
    // its own content (house / windmill / empty gap), along-road phase
    // offset, and depth-from-road offset from a small pseudo-random hash
    // of its own slot index -- computed entirely at elaboration time
    // (gi is a genvar, so every "random" value below is just a distinct
    // compile-time constant per instance, at zero runtime hardware
    // cost). That's what keeps this from reading as a straight, evenly
    // spaced "wall" of buildings: some slots sit closer, some further,
    // some are houses, some are windmills, some are left empty as
    // breathing room. Distance-scaling and curve-following reuse the
    // exact same wrap_ty/curve_shift_at math as the treeline, so the
    // whole village still bends smoothly with the road and keeps
    // scrolling forever.
    // ------------------------------------------------------------------
    localparam NUM_SCENERY_SIDE        = 5; // reduced from 9 -- see note below about compile stability
    localparam [9:0] SCEN_Y_STEP       = (TREE_Y_MAX - TREE_Y_MIN) / NUM_SCENERY_SIDE; // 40
    localparam [9:0] SCEN_RIGHT_OFF    = SCEN_Y_STEP >> 1; // stagger right side vs left
    localparam [9:0] SCEN_BASE_GAP     = 10'd22; // baseline extra gap beyond the treeline
    localparam [9:0] HOUSE_BASE_HALF   = 10'd6;  // wall half-width at the horizon (small/far away)
    localparam       HOUSE_SCALE_SHIFT = 4;
    localparam [9:0] WINDMILL_BASE_HALF   = 10'd3; // tower half-width at the horizon
    localparam       WINDMILL_SCALE_SHIFT = 4;

    wire [2:0] hp_left  [0:NUM_SCENERY_SIDE-1];
    wire [2:0] hp_right [0:NUM_SCENERY_SIDE-1];
    wire [2:0] wp_left  [0:NUM_SCENERY_SIDE-1];
    wire [2:0] wp_right [0:NUM_SCENERY_SIDE-1];

    generate
        for (gi = 0; gi < NUM_SCENERY_SIDE; gi = gi + 1) begin : gen_left_scenery
            // Elaboration-time pseudo-random hash of this slot's index --
            // just an LCG-flavored constant expression, unique per gi.
            localparam [15:0] SEED       = (gi * 16'd40503) + 16'd18041;
            localparam [2:0]  TYPE_SEL   = SEED[2:0];   // 0-3 house, 4/5/7 windmill, 6 empty
            localparam [9:0]  PHASE_JIT  = {6'd0, SEED[6:3]};   // 0..15 px along-road jitter
            localparam [9:0]  GAP_JIT    = {7'd0, SEED[9:7]} << 2; // 0..28 px depth jitter
            localparam        IS_HOUSE   = (TYPE_SEL <= 3'd3);
            localparam        IS_WINDMILL = (TYPE_SEL == 3'd4) || (TYPE_SEL == 3'd5) || (TYPE_SEL == 3'd7);

            localparam [9:0] base_sy = TREE_Y_MIN + (SCEN_Y_STEP >> 1) + gi * SCEN_Y_STEP + PHASE_JIT;

            wire [9:0] this_sy    = wrap_ty(base_sy, scroll_y);
            wire [9:0] this_dy    = this_sy - TREE_Y_MIN;
            wire [9:0] this_rhalf = ROAD_HALF_FAR + (this_dy >> 1);

            wire signed [13:0] this_cshift = curve_shift_at(curve_amount, this_dy);
            wire signed [13:0] this_censig = $signed({1'b0, ROAD_CENTER_X}) + this_cshift;
            wire [9:0] this_center = clamp_center_x(this_censig, 10'd5);

            wire [9:0] this_house_half = HOUSE_BASE_HALF    + (this_dy >> HOUSE_SCALE_SHIFT);
            wire [9:0] this_wind_half  = WINDMILL_BASE_HALF + (this_dy >> WINDMILL_SCALE_SHIFT);
            // The windmill's actual visible footprint is its BLADE SWEEP,
            // not just this_wind_half -- windmill_at is called below with
            // a blade radius of (this_wind_half*2)+8, so that's what has
            // to be reserved here too, or the blades reach back past the
            // gap and swipe into the treeline.
            wire [9:0] this_wind_blade_r = (this_wind_half <<< 1) + 10'd8;
            wire [9:0] this_obj_half   = IS_HOUSE ? this_house_half : this_wind_blade_r;

            wire [9:0] this_sx = this_center - this_rhalf - TREE_GAP - SCEN_BASE_GAP - GAP_JIT - this_obj_half;

            assign hp_left[gi] = IS_HOUSE ? house_at(x, y, this_sx, this_sy,
                                                this_house_half[4:0], (this_house_half[4:0] >> 1) + 5'd2,
                                                (this_house_half[4:0] >> 1) + 5'd3, this_house_half[4:0] + 5'd2)
                                           : 3'b000;
            assign wp_left[gi] = IS_WINDMILL ? windmill_at(x, y, this_sx, this_sy,
                                                this_wind_half[4:0], (this_wind_half[4:0] <<< 1) + 5'd6,
                                                this_wind_blade_r[4:0], windmill_spin)
                                              : 3'b000;
        end

        for (gi = 0; gi < NUM_SCENERY_SIDE; gi = gi + 1) begin : gen_right_scenery
            localparam [15:0] SEED       = (gi * 16'd40503) + 16'd52237;
            localparam [2:0]  TYPE_SEL   = SEED[2:0];
            localparam [9:0]  PHASE_JIT  = {6'd0, SEED[6:3]};
            localparam [9:0]  GAP_JIT    = {7'd0, SEED[9:7]} << 2;
            localparam        IS_HOUSE   = (TYPE_SEL <= 3'd3);
            localparam        IS_WINDMILL = (TYPE_SEL == 3'd4) || (TYPE_SEL == 3'd5) || (TYPE_SEL == 3'd7);

            localparam [9:0] base_sy = TREE_Y_MIN + (SCEN_Y_STEP >> 1) + SCEN_RIGHT_OFF + gi * SCEN_Y_STEP + PHASE_JIT;

            wire [9:0] this_sy    = wrap_ty(base_sy, scroll_y);
            wire [9:0] this_dy    = this_sy - TREE_Y_MIN;
            wire [9:0] this_rhalf = ROAD_HALF_FAR + (this_dy >> 1);

            wire signed [13:0] this_cshift = curve_shift_at(curve_amount, this_dy);
            wire signed [13:0] this_censig = $signed({1'b0, ROAD_CENTER_X}) + this_cshift;
            wire [9:0] this_center = clamp_center_x(this_censig, 10'd5);

            wire [9:0] this_house_half = HOUSE_BASE_HALF    + (this_dy >> HOUSE_SCALE_SHIFT);
            wire [9:0] this_wind_half  = WINDMILL_BASE_HALF + (this_dy >> WINDMILL_SCALE_SHIFT);
            wire [9:0] this_wind_blade_r = (this_wind_half <<< 1) + 10'd8;
            wire [9:0] this_obj_half   = IS_HOUSE ? this_house_half : this_wind_blade_r;

            wire [9:0] this_sx = this_center + this_rhalf + TREE_GAP + SCEN_BASE_GAP + GAP_JIT + this_obj_half;

            assign hp_right[gi] = IS_HOUSE ? house_at(x, y, this_sx, this_sy,
                                                this_house_half[4:0], (this_house_half[4:0] >> 1) + 5'd2,
                                                (this_house_half[4:0] >> 1) + 5'd3, this_house_half[4:0] + 5'd2)
                                            : 3'b000;
            assign wp_right[gi] = IS_WINDMILL ? windmill_at(x, y, this_sx, this_sy,
                                                this_wind_half[4:0], (this_wind_half[4:0] <<< 1) + 5'd6,
                                                this_wind_blade_r[4:0], windmill_spin)
                                               : 3'b000;
        end
    endgenerate

    // Same reduction idiom as the trees above -- flatten all instances
    // down to the category flags the color mux needs.
    reg house_wall_lit, house_wall_shadow, house_roof, house_window, house_door;
    reg windmill_tower_lit, windmill_tower_shadow, windmill_blade, windmill_hub;
    integer hi_i;
    always @(*) begin
        house_wall_lit    = 1'b0;
        house_wall_shadow = 1'b0;
        house_roof        = 1'b0;
        house_window      = 1'b0;
        house_door        = 1'b0;
        windmill_tower_lit    = 1'b0;
        windmill_tower_shadow = 1'b0;
        windmill_blade         = 1'b0;
        windmill_hub            = 1'b0;
        for (hi_i = 0; hi_i < NUM_SCENERY_SIDE; hi_i = hi_i + 1) begin
            if (hp_left[hi_i] == 3'b001 || hp_right[hi_i] == 3'b001) house_wall_lit    = 1'b1;
            if (hp_left[hi_i] == 3'b010 || hp_right[hi_i] == 3'b010) house_wall_shadow = 1'b1;
            if (hp_left[hi_i] == 3'b011 || hp_right[hi_i] == 3'b011) house_roof        = 1'b1;
            if (hp_left[hi_i] == 3'b100 || hp_right[hi_i] == 3'b100) house_window      = 1'b1;
            if (hp_left[hi_i] == 3'b101 || hp_right[hi_i] == 3'b101) house_door        = 1'b1;

            if (wp_left[hi_i] == 3'b001 || wp_right[hi_i] == 3'b001) windmill_tower_lit    = 1'b1;
            if (wp_left[hi_i] == 3'b010 || wp_right[hi_i] == 3'b010) windmill_tower_shadow = 1'b1;
            if (wp_left[hi_i] == 3'b011 || wp_right[hi_i] == 3'b011) windmill_blade        = 1'b1;
            if (wp_left[hi_i] == 3'b100 || wp_right[hi_i] == 3'b100) windmill_hub          = 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // Curve direction detection: no longer drawn as a diamond sign out
    // in the 3D world -- replaced by a top-center HUD indicator (see
    // the "curve direction HUD" block further down, near the score
    // panel) so the player gets a clear on-screen "which way" cue
    // instead of a roadside board that's easy to miss. These wires just
    // say WHEN a turn is coming and WHICH way, driven off `phase`/
    // `seg_dist` declared above -- the HUD block reuses them directly.
    // ------------------------------------------------------------------
    localparam [19:0] SIGN_WINDOW = 20'd220;

    wire sign_upcoming_right = (phase == PHASE_STRAIGHT_1);
    wire sign_upcoming_left  = (phase == PHASE_STRAIGHT_2);
    wire sign_timing         = (seg_dist >= (SEGMENT_LENGTH - SIGN_WINDOW));
    wire sign_visible        = (sign_upcoming_right || sign_upcoming_left) && sign_timing;

	 // ================= Background ROM =================
	wire [16:0] bg_addr;
	wire [11:0] bg_pixel;

    // ------------------------------------------------------------------
    // Parallax: shift which column of the mountain image we sample by a
    // fraction of the car's steering offset, so the background visibly
    // slides when you steer left/right -- like the "camera" panning
    // with you, even though the mountains are just a flat image ROM.
    // >>4 keeps the shift subtle (mountains are "far away" so they
    // should move less than nearby objects); car_offset's full range is
    // -256..256, so this gives roughly -16..16 px of background pan.
    // ------------------------------------------------------------------
    localparam PARALLAX_SHIFT_DIV = 4; // bigger = subtler background movement

    wire signed [10:0] parallax_shift = car_offset >>> PARALLAX_SHIFT_DIV;

    wire signed [10:0] bg_sample_x_signed = $signed({1'b0, x}) - parallax_shift;
    wire [9:0] bg_sample_x = (bg_sample_x_signed < 11'sd0)          ? (bg_sample_x_signed + 11'sd640) :
                             (bg_sample_x_signed >= 11'sd640)        ? (bg_sample_x_signed - 11'sd640) :
                                                                       bg_sample_x_signed[9:0];

	// Gambar VGA 640 -- kolom sampling sekarang digeser oleh parallax_shift
	assign bg_addr = (y < 120) ? (y * 640 + {7'd0, bg_sample_x}) : 17'd0;

	background_rom bg_rom (
    .address(bg_addr),
    .clock(clk),
    .q(bg_pixel)
);

    // Second read-only instance of the *same* background image, always
    // sampling its very last row (the horizon line) at the current
    // column x -- regardless of what row y we're actually drawing.
    // This gives us, for every column, the exact mountain/sky color
    // that sits right above the ground at that x, so the ground can
    // fade into it instead of hard-cutting to a hand-picked color.
    // Also shifted by the same parallax offset so the horizon-fade
    // color stays consistent with what the sky above it is doing.
    wire [16:0] bg_horizon_addr;
    wire [11:0] bg_horizon_pixel;

    assign bg_horizon_addr = 17'd119 * 17'd640 + {7'd0, bg_sample_x};

    background_rom bg_rom_horizon (
        .address(bg_horizon_addr),
        .clock(clk),
        .q(bg_horizon_pixel)
    );

    wire [7:0] horizon_r = {bg_horizon_pixel[11:8], bg_horizon_pixel[11:8]};
    wire [7:0] horizon_g = {bg_horizon_pixel[7:4],  bg_horizon_pixel[7:4]};
    wire [7:0] horizon_b = {bg_horizon_pixel[3:0],  bg_horizon_pixel[3:0]};

    // Fade window just below the horizon line: at y==120 the ground is
    // (almost) entirely the sampled mountain/sky color, easing to 100%
    // ground color by y==135. HORIZON_BLEND_H is kept at 16 so the
    // fade weight maps directly onto blend4's 0-15 range with no
    // division needed.
    localparam HORIZON_BLEND_H = 10'd16;
    wire in_horizon_blend = (y >= 10'd120) && (y < (10'd120 + HORIZON_BLEND_H));
    wire [3:0] horizon_blend_w = (y - 10'd120);
	 // ================================================

 

    // Back layer - 5 peaks spanning the width, hazy/lighter color
    localparam MB1_PX = 60,  MB1_PY = 45, MB1_HB = 70;
    localparam MB2_PX = 180, MB2_PY = 35, MB2_HB = 90;
    localparam MB3_PX = 330, MB3_PY = 50, MB3_HB = 80;
    localparam MB4_PX = 470, MB4_PY = 38, MB4_HB = 100;
    localparam MB5_PX = 590, MB5_PY = 55, MB5_HB = 60;

    localparam MTN_HORIZON = 120; // shared base line = top of the road

    wire [9:0] mb1_dx, mb2_dx, mb3_dx, mb4_dx, mb5_dx;
    assign mb1_dx = (x >= MB1_PX) ? (x - MB1_PX) : (MB1_PX - x);
    assign mb2_dx = (x >= MB2_PX) ? (x - MB2_PX) : (MB2_PX - x);
    assign mb3_dx = (x >= MB3_PX) ? (x - MB3_PX) : (MB3_PX - x);
    assign mb4_dx = (x >= MB4_PX) ? (x - MB4_PX) : (MB4_PX - x);
    assign mb5_dx = (x >= MB5_PX) ? (x - MB5_PX) : (MB5_PX - x);

   
    wire mb1, mb2, mb3, mb4, mb5;

    assign mb1 = (y >= MB1_PY) && (mb1_dx * (MTN_HORIZON - MB1_PY) <= MB1_HB * (y - MB1_PY));
    assign mb2 = (y >= MB2_PY) && (mb2_dx * (MTN_HORIZON - MB2_PY) <= MB2_HB * (y - MB2_PY));
    assign mb3 = (y >= MB3_PY) && (mb3_dx * (MTN_HORIZON - MB3_PY) <= MB3_HB * (y - MB3_PY));
    assign mb4 = (y >= MB4_PY) && (mb4_dx * (MTN_HORIZON - MB4_PY) <= MB4_HB * (y - MB4_PY));
    assign mb5 = (y >= MB5_PY) && (mb5_dx * (MTN_HORIZON - MB5_PY) <= MB5_HB * (y - MB5_PY));

    wire mtn_back1;
    assign mtn_back1 = mb1 || mb2 || mb3 || mb4 || mb5;

    assign road_area = (y >= 10'd120) &&
                       (x >= road_left) &&
                       (x <= road_right);

    assign left_solid_line = road_area &&
                             (x >= road_left + line_margin) &&
                             (x <= road_left + line_margin + line_thick);

    assign right_solid_line = road_area &&
                              (x >= road_right - line_margin - line_thick) &&
                              (x <= road_right - line_margin);

    assign center_dash = road_area &&
                         (x >= curved_center_x - (line_thick >> 1)) &&
                         (x <= curved_center_x + (line_thick >> 1)) &&
                         (anim_y[5] == 1'b0);
   
    assign trotoar_left  = (y >= 10'd120) &&
                            (x >= road_left - TROTOAR_WIDTH) &&
                            (x <  road_left);

    assign trotoar_right = (y >= 10'd120) &&
                            (x >  road_right) &&
                            (x <= road_right + TROTOAR_WIDTH);

    // Stripe pattern for the curb: toggles black/white every few rows
    // of world-scroll (anim_y), same "cheap XOR/bit-toggle" idiom used
    // for the dashed center line and asphalt speckle above, so the
    // stripes visibly scroll toward the camera along with everything
    // else. A little wider-spaced (bit 4) than the center dash (bit 5)
    // so the two don't look identical.
    wire trotoar_stripe = anim_y[4];

localparam CAR_W = 10'd128;
    localparam CAR_H = 10'd128;
    localparam CAR_X_BASE = 10'd256; // (640 - 128) / 2 = 256 (posisi tengah)
    localparam CAR_Y = 10'd330; // Posisi vertikal di bagian bawah jalan

    // Mobil sekarang bergerak bebas, terpisah total dari lengkungan
    // jalan: tahan steer_left/steer_right untuk menggeser mobil, lepas
    // untuk berhenti persis di posisi itu (tidak otomatis balik ke
    // tengah). Dibatasi CAR_OFFSET_MIN/MAX supaya mobil tidak pernah
    // bergeser sampai keluar dari sisi layar (yang bisa merusak
    // perhitungan alamat ROM di bawah).
    localparam signed CAR_STEP_BASE    = 3;    // kecepatan geser dasar per frame saat ditahan (di skor 0)
    localparam signed [10:0] CAR_OFFSET_MIN  = -11'sd256; // batas mutlak: CAR_X minimum = 256 + (-256) = 0
    localparam signed [10:0] CAR_OFFSET_MAX  =  11'sd256; // batas mutlak: CAR_X maksimum = 256 + 256 = 512

    // ------------------------------------------------------------------
    // Hitbox margins untuk mobil (dipindah ke sini supaya bisa dipakai
    // oleh perhitungan batas jalan-dinamis di bawah -- nilainya sama
    // seperti yang dipakai nanti untuk collision box).
    // ------------------------------------------------------------------
    localparam CAR_HITBOX_MARGIN_TOP    = 10'd60; // biggest one -- most of the empty space is above the roof
    localparam CAR_HITBOX_MARGIN_BOTTOM = 10'd5;
    localparam CAR_HITBOX_MARGIN_SIDE   = 10'd12;

    reg signed [10:0] car_offset;

    // ------------------------------------------------------------------
    // Batas jalan dinamis untuk mobil: bukan cuma batas layar tetap,
    // tapi batas ASLI tepi jalan (mengikuti lengkungan curve_amount)
    // pada baris tempat bagian TERSEMPIT hitbox mobil berada. Bagian
    // atas hitbox (CAR_Y + CAR_HITBOX_MARGIN_TOP) dipakai sebagai
    // acuan karena jalan makin sempit semakin dekat ke horizon --
    // itulah baris paling ketat yang harus dijaga supaya mobil tidak
    // pernah keluar dari aspal, persis seperti perhitungan jalan utama
    // dan rintangan di atas.
    // ------------------------------------------------------------------
    wire [9:0] dy_car         = (CAR_Y + CAR_HITBOX_MARGIN_TOP) - 10'd120;
    wire signed [13:0] curve_shift_car     = curve_shift_at(curve_amount, dy_car);
    wire signed [13:0] center_x_car_signed = $signed({1'b0, ROAD_CENTER_X}) + curve_shift_car;
    wire [9:0] road_half_car  = ROAD_HALF_FAR + (dy_car >> 1);
    wire [9:0] curved_center_x_car = clamp_center_x(center_x_car_signed, road_half_car);
    wire [9:0] road_left_car  = curved_center_x_car - road_half_car;
    wire [9:0] road_right_car = curved_center_x_car + road_half_car;

    // Ubah batas tepi jalan itu menjadi rentang car_offset yang boleh
    // dipakai, supaya sisi kiri/kanan HITBOX mobil (bukan sprite penuh)
    // selalu berada di antara road_left_car dan road_right_car. Tetap
    // dijepit lagi ke CAR_OFFSET_MIN/MAX supaya mobil juga tidak pernah
    // keluar dari tepi layar walau perhitungan jalan menghasilkan
    // sesuatu yang tidak terduga.
    wire signed [10:0] offset_min_road = $signed({1'b0, road_left_car})  - $signed({1'b0, CAR_HITBOX_MARGIN_SIDE}) - $signed({1'b0, CAR_X_BASE});
    wire signed [10:0] offset_max_road = $signed({1'b0, road_right_car}) + $signed({1'b0, CAR_HITBOX_MARGIN_SIDE}) - $signed({1'b0, CAR_W}) - $signed({1'b0, CAR_X_BASE});

    wire signed [10:0] car_offset_min_eff = (offset_min_road > CAR_OFFSET_MIN) ? offset_min_road : CAR_OFFSET_MIN;
    wire signed [10:0] car_offset_max_eff = (offset_max_road < CAR_OFFSET_MAX) ? offset_max_road : CAR_OFFSET_MAX;

    // ------------------------------------------------------------------
    // BUG FIX: on a sharp curve, road_left_car and road_right_car both
    // shift sideways together, which can squeeze the gap between
    // car_offset_min_eff and car_offset_max_eff down to less than the
    // car's own hitbox width -- sometimes even to a single point. When
    // that happens the car gets pinned wherever the clamp lands, and
    // that pinned spot can still overlap an obstacle lane, so the
    // player gets hit even though the obstacle still looks "far away"
    // on screen: the real problem is there was never enough room left
    // to steer out of the way in time, not that the collision box is
    // wrong.
    //
    // Fix: if the clamped window is narrower than MIN_DRIVABLE_WIDTH,
    // re-center a window of at least that width around the same
    // midpoint, still bounded by the absolute screen limits
    // (CAR_OFFSET_MIN/MAX) so the car can never be pushed off-screen.
    // ------------------------------------------------------------------
    localparam signed [10:0] MIN_DRIVABLE_WIDTH = 11'sd170; // car hitbox (104px) + real dodge room

    wire signed [10:0] eff_width = car_offset_max_eff - car_offset_min_eff;
    wire signed [10:0] eff_mid   = (car_offset_max_eff + car_offset_min_eff) >>> 1;

    wire signed [10:0] widened_min_raw = eff_mid - (MIN_DRIVABLE_WIDTH >>> 1);
    wire signed [10:0] widened_max_raw = eff_mid + (MIN_DRIVABLE_WIDTH >>> 1);

    wire signed [10:0] widened_min = (widened_min_raw < CAR_OFFSET_MIN) ? CAR_OFFSET_MIN : widened_min_raw;
    wire signed [10:0] widened_max = (widened_max_raw > CAR_OFFSET_MAX) ? CAR_OFFSET_MAX : widened_max_raw;

    wire signed [10:0] car_offset_min_final = (eff_width < MIN_DRIVABLE_WIDTH) ? widened_min : car_offset_min_eff;
    wire signed [10:0] car_offset_max_final = (eff_width < MIN_DRIVABLE_WIDTH) ? widened_max : car_offset_max_eff;

    always @(posedge clk) begin
        if (reset) begin
            car_offset <= 11'sd0;
        end else if (x == 10'd0 && y == 10'd0 && !game_over && game_started) begin   // sekali per frame
            if (steer_left && !steer_right) begin
                if ((car_offset - CAR_STEP) < car_offset_min_final)
                    car_offset <= car_offset_min_final;
                else
                    car_offset <= car_offset - CAR_STEP;
            end else if (steer_right && !steer_left) begin
                if ((car_offset + CAR_STEP) > car_offset_max_final)
                    car_offset <= car_offset_max_final;
                else
                    car_offset <= car_offset + CAR_STEP;
            end else begin
                // Tidak ada tombol ditahan -> diam di tempat, tapi tetap
                // dijepit ulang kalau-kalau lengkungan jalan baru saja
                // menyempit di bawah posisi mobil yang sedang diam.
                if (car_offset < car_offset_min_final)
                    car_offset <= car_offset_min_final;
                else if (car_offset > car_offset_max_final)
                    car_offset <= car_offset_max_final;
            end
        end
    end

    wire signed [10:0] car_x_signed;
    assign car_x_signed = $signed({1'b0, CAR_X_BASE}) + car_offset;

    wire [9:0] CAR_X;
    assign CAR_X = car_x_signed[9:0];

    // ------------------------------------------------------------------
    // Turn sprite select: swap the whole car sprite for a pre-drawn
    // "serong kiri" / "serong kanan" (angled) version based on which
    // way the ROAD is currently curving -- not based on the player's
    // steer input. curve_amount (declared up with the road-curve logic)
    // is >0 while the road bends right and <0 while it bends left, and
    // eases smoothly through 0 on straight sections, so this naturally
    // follows the curve instead of the steer keys.
    // TURN_SPRITE_DEADZONE avoids flicker right around dead-straight,
    // where curve_amount briefly sits at/near 0 between phases.
    // ------------------------------------------------------------------
    localparam signed TURN_SPRITE_DEADZONE = 1;

    wire turn_left_active  = curve_amount < -TURN_SPRITE_DEADZONE;
    wire turn_right_active = curve_amount >  TURN_SPRITE_DEADZONE;

    // Plain (non-sheared) column of the pixel under test, relative to
    // the car sprite's left edge -- all three sprites share the same
    // box/addressing, only the ROM contents differ.
    wire signed [10:0] col_rel = $signed({1'b0, x}) - $signed({1'b0, CAR_X});
    wire is_car_col_valid = (col_rel >= 11'sd0) && (col_rel < $signed({1'b0, CAR_W}));

    wire is_car_area = is_car_col_valid && (y >= CAR_Y) && (y < CAR_Y + CAR_H);

    // Perhitungan alamat memori (14-bit bus untuk 16384 data)
    // Perkalian dengan 128 secara otomatis akan dioptimasi menjadi operasi geser bit (shift) oleh FPGA
    wire [13:0] rom_addr = ((y - CAR_Y) * CAR_W) + col_rel[9:0];

    // ------------------------------------------------------------------
    // Obstacle (rintangan): a vehicle -- randomly either a car or a
    // motorcycle at each respawn -- that travels from just below the
    // horizon down to the player's row, scaling up the same way the
    // road itself widens with dy, then loops back to the horizon with
    // a new random lane AND a freshly re-rolled vehicle type. Drawn as
    // a colored silhouette viewed from behind (same style family as
    // the trees/houses) instead of a photo sprite, so it needs no new
    // ROM/image asset.
    // ------------------------------------------------------------------
    localparam OBS_Y_START      = 10'd130;          // spawn row, just past the horizon
    localparam OBS_Y_END        = CAR_Y + CAR_H;     // once it reaches here it has "passed" the player
    // Diperkecil sedikit dari sebelumnya (6/6, shift 3) supaya rintangan
    // tidak terlalu besar/menakutkan dan tumbuh lebih halus saat mendekat.
    localparam OBS_BASE_HALF_W  = 10'd4;   // half-width at the horizon (small/far away)
    localparam OBS_BASE_HALF_H  = 10'd4;   // half-height at the horizon
    localparam OBS_SCALE_SHIFT  = 4;       // how fast it grows with distance (smaller = grows faster)

    // ------------------------------------------------------------------
    // Difficulty scaling: obstacles approach faster as the score goes
    // up. One extra speed step every OBSTACLE_SPEEDUP_SCORE points,
    // capped at OBSTACLE_SPEED_MAX_BONUS so it never becomes literally
    // impossible / doesn't overflow anything downstream.
    // ------------------------------------------------------------------
    localparam [15:0] OBSTACLE_SPEEDUP_SCORE   = 16'd50; // score points needed per speed-up step
    localparam [9:0]  OBSTACLE_SPEED_MAX_BONUS = 10'd7;  // hard cap on how much speed can be added

    // ------------------------------------------------------------------
    // BUG FIX: this used to be `score / OBSTACLE_SPEEDUP_SCORE`, a plain
    // 16-bit / 50 divide. 50 isn't a power of 2 either, so that's a full
    // divider circuit synthesized in hardware -- another contributor to
    // the same Analysis & Synthesis blow-up as wrap_ty's old `%` above.
    // Since the result is capped at just OBSTACLE_SPEED_MAX_BONUS (7)
    // tiers, we only ever care whether score has crossed 7 fixed
    // thresholds (50, 100, 150, ... 350) -- a chain of plain comparators
    // gives the exact same answer for a fraction of the synthesis cost.
    // ------------------------------------------------------------------
    wire [9:0] obstacle_speed_bonus =
        (score >= 16'd350) ? 10'd7 :
        (score >= 16'd300) ? 10'd6 :
        (score >= 16'd250) ? 10'd5 :
        (score >= 16'd200) ? 10'd4 :
        (score >= 16'd150) ? 10'd3 :
        (score >= 16'd100) ? 10'd2 :
        (score >= 16'd50)  ? 10'd1 :
                              10'd0;
    wire [9:0] OBSTACLE_SPEED = world_scroll_speed; // same rate the road/trees now scroll at, so everything speeds up together

    // CAR_STEP naik pakai bonus YANG SAMA persis dengan OBSTACLE_SPEED
    // (bukan formula terpisah), jadi keduanya selalu "mengejar" satu
    // sama lain -- rasio kecepatan geser mobil vs kecepatan mendekat
    // rintangan tetap sama dari skor 0 sampai skor tinggi, bukan makin
    // berat sepihak karena rintangan makin cepat tapi mobil tidak.
    wire signed [10:0] CAR_STEP = $signed({1'b0, CAR_STEP_BASE}) + $signed({1'b0, obstacle_speed_bonus});

    reg [9:0] obstacle_y;
    reg [1:0] obstacle_lane;   // 0 = kiri, 1 = tengah, 2 = kanan (3 tidak dipakai)
    reg       obstacle_is_car; // 0 = motor, 1 = mobil -- re-rolled at every respawn
    reg [7:0] obstacle_rand;   // 8-bit LFSR, sampled at respawn for a pseudo-random lane
    reg       obstacle_overlap_prev; // overlap state one frame ago, for edge detection

    // ------------------------------------------------------------------
    // Lives: 3 starting chances, up to LIVES_MAX -- extra lives now come
    // from catching the star pickup (see below) instead of an automatic
    // score threshold. Each hit costs one life AND LIFE_LOST_PENALTY
    // points (which also naturally lowers obstacle_speed_bonus/
    // difficulty since that's derived straight from `score` -- no
    // separate "slow back down" logic needed, it falls out of the
    // existing difficulty formula for free). Only the hit that takes
    // the LAST life actually sets game_over. Widened to 3 bits (was 2)
    // so lives can go past 3 once bonus lives start coming in.
    // ------------------------------------------------------------------
    localparam [2:0]  LIVES_START       = 3'd3;
    localparam [2:0]  LIVES_MAX         = 3'd5;
    localparam [15:0] LIFE_LOST_PENALTY = 16'd10;

    reg [2:0] lives;
    initial lives = LIVES_START;

    // ------------------------------------------------------------------
    // Bonus-life pickup: a star that travels down the road exactly like
    // an obstacle (same lane/perspective/speed math), but drawn as a
    // gold 4-point sparkle instead of a hazard drum so the player can
    // instantly tell it's good, not bad. Touching it with the car grants
    // +1 life (capped at LIVES_MAX) instead of costing one. Spawns on
    // its own timer, independent of the obstacle's spawn/respawn cycle.
    // ------------------------------------------------------------------
    localparam [15:0] BONUS_SPAWN_INTERVAL = 16'd480; // frames between star spawns (~8s @ 60fps)
    localparam [9:0]  BONUS_BASE_R         = 10'd5;   // star radius at the horizon (small/far away)
    localparam        BONUS_SCALE_SHIFT    = 4;       // how fast it grows with distance (same idea as the obstacle)

    reg        bonus_active;
    reg [9:0]  bonus_y;
    reg [1:0]  bonus_lane;
    reg        bonus_overlap_prev;
    reg [15:0] bonus_spawn_timer;

    initial bonus_active       = 1'b0;
    initial bonus_y            = OBS_Y_START;
    initial bonus_lane         = 2'd1;
    initial bonus_overlap_prev = 1'b0;
    initial bonus_spawn_timer  = BONUS_SPAWN_INTERVAL;

    initial obstacle_y            = OBS_Y_START;
    initial obstacle_lane         = 2'd1;
    initial obstacle_is_car       = 1'b1;
    initial obstacle_rand         = 8'hA5; // must be non-zero for the LFSR to run
    initial obstacle_overlap_prev = 1'b0;
    initial score                 = 16'd0;
    initial game_over              = 1'b0;

    wire obstacle_reached_end = (obstacle_y + OBSTACLE_SPEED) >= OBS_Y_END;

    // ------------------------------------------------------------------
    // NOTE: a plain "+1 every frame" counter sampled only at respawn
    // time doesn't actually give varied lanes -- obstacles always take
    // the same number of frames to cross the road (until the speed
    // changes at a difficulty tier), so the sampled low bits advance by
    // the SAME fixed amount every single respawn. If that amount is a
    // multiple of 4, the picked lane freezes on one value forever --
    // which is exactly why obstacles kept spawning in the same spot.
    // An LFSR doesn't have this problem: advancing it by a fixed number
    // of steps each respawn still visits many different pseudo-random
    // states instead of looping back to the same one.
    // ------------------------------------------------------------------
    // 2 separate bit-pairs XORed together before folding, so the lane
    // choice depends on more of the LFSR's state instead of just its
    // lowest 2 bits.
    wire [1:0] next_lane_raw = obstacle_rand[1:0] ^ obstacle_rand[6:5];
    wire [1:0] next_lane     = (next_lane_raw == 2'd3) ? 2'd1 : next_lane_raw;

    // Random vehicle type for the next spawn -- a different LFSR bit
    // than the lane picker above uses, so the two choices don't end up
    // correlated with each other.
    wire next_is_car = obstacle_rand[7] ^ obstacle_rand[2];

    // Same perspective math the road itself uses (curve_base/curved
    // center/half-width), just evaluated at the obstacle's OWN distance
    // (dy_obs) instead of the current scanline's dy -- it's a flat box,
    // so one size/position per frame is enough, no per-row recompute.
    wire [9:0] dy_obs         = obstacle_y - 10'd120;
    wire signed [13:0] curve_shift_obs = curve_shift_at(curve_amount, dy_obs);
    wire signed [13:0] center_x_obs_signed = $signed({1'b0, ROAD_CENTER_X}) + curve_shift_obs;
    wire [9:0] road_half_obs  = ROAD_HALF_FAR + (dy_obs >> 1);

    // Lane offset as a fraction of the road's current half-width at
    // this distance, so the obstacle stays inside the road at any
    // point along its approach instead of drifting into the grass.
    wire signed [10:0] lane_offset = (obstacle_lane == 2'd0) ? -$signed({1'b0, road_half_obs}) >>> 1 :
                                      (obstacle_lane == 2'd2) ?  $signed({1'b0, road_half_obs}) >>> 1 :
                                                                  11'sd0;

    wire signed [10:0] obstacle_x_signed = center_x_obs_signed + lane_offset;
    wire [9:0] obstacle_x = clamp_center_x(obstacle_x_signed, 10'd25); // 25 > OBS_MAX_HALF_W, safe margin

    // Motorcycle gets a narrower box than a car -- it's a visibly
    // narrower vehicle, so a narrower hitbox to match is fair.
    wire [9:0] obs_half_w = obstacle_is_car
        ? (OBS_BASE_HALF_W + (dy_obs >> OBS_SCALE_SHIFT) + 10'd2)
        : (((OBS_BASE_HALF_W + (dy_obs >> OBS_SCALE_SHIFT)) >> 1) + 10'd1);
    wire [9:0] obs_half_h = OBS_BASE_HALF_H + (dy_obs >> OBS_SCALE_SHIFT);

    wire signed [10:0] obs_col_rel = $signed({1'b0, x}) - $signed({1'b0, obstacle_x});
    wire signed [10:0] obs_row_rel = $signed({1'b0, y}) - $signed({1'b0, obstacle_y});

    wire is_obstacle_area = (obs_col_rel >= -$signed({1'b0, obs_half_w})) && (obs_col_rel <= $signed({1'b0, obs_half_w})) &&
                             (obs_row_rel >= -$signed({1'b0, obs_half_h})) && (obs_row_rel <= $signed({1'b0, obs_half_h}));

    // ------------------------------------------------------------------
    // Vehicle silhouette, viewed from behind (like a car or motorbike
    // ahead of the player on the road). All region tests below reuse
    // obs_col_rel/obs_row_rel (plain signed subtractions, already
    // computed above) -- no multiplication needed, so there's no width-
    // truncation risk here the way there was for the circular shapes
    // elsewhere in this file (star/boulder/person).
    //
    // Car: body fill, a darker rear-window band across the top third,
    // and two bright taillights at the bottom corners.
    // Motorcycle: a small helmet band at the very top, body/seat below
    // it, and a single centered taillight at the bottom.
    // ------------------------------------------------------------------
    wire signed [10:0] obs_top   = -$signed({1'b0, obs_half_h});
    wire signed [10:0] obs_bot   =  $signed({1'b0, obs_half_h});
    wire signed [10:0] obs_left  = -$signed({1'b0, obs_half_w});
    wire signed [10:0] obs_right =  $signed({1'b0, obs_half_w});

    wire [9:0] obs_band_h = (obs_half_h > 10'd1) ? (obs_half_h >> 1) : 10'd1; // ~half the box height

    // Car-only regions
    wire obs_car_window = obstacle_is_car &&
                          (obs_row_rel >= obs_top) &&
                          (obs_row_rel <= obs_top + $signed({1'b0, obs_band_h}));

    wire [9:0] obs_car_light_w = (obs_half_w > 10'd3) ? (obs_half_w >> 2) : 10'd1;
    wire obs_car_taillight = obstacle_is_car &&
                             (obs_row_rel >= obs_bot - $signed({1'b0, obs_band_h})) &&
                             (obs_row_rel <= obs_bot) &&
                             (((obs_col_rel >= obs_left) && (obs_col_rel <= obs_left + $signed({1'b0, obs_car_light_w}))) ||
                              ((obs_col_rel <= obs_right) && (obs_col_rel >= obs_right - $signed({1'b0, obs_car_light_w}))));

    // Motorcycle-only regions
    wire obs_moto_helmet = !obstacle_is_car &&
                           (obs_row_rel >= obs_top) &&
                           (obs_row_rel <= obs_top + $signed({1'b0, obs_band_h}) - 11'sd1);

    wire [9:0] obs_moto_light_w = (obs_half_w > 10'd1) ? (obs_half_w >> 1) : 10'd1;
    wire obs_moto_taillight = !obstacle_is_car &&
                              (obs_row_rel >= obs_bot - 11'sd1) &&
                              (obs_row_rel <= obs_bot) &&
                              (obs_col_rel >= -$signed({1'b0, obs_moto_light_w})) &&
                              (obs_col_rel <=  $signed({1'b0, obs_moto_light_w}));

    // ------------------------------------------------------------------
    // Bonus star pickup: exactly the same perspective/lane math the
    // obstacle uses (own distance dy_bonus -> curve_shift -> lane
    // offset -> clamp), just evaluated for bonus_y/bonus_lane instead
    // -- so it travels down the road, follows the curve, and scales
    // with distance the same way the obstacle does.
    // ------------------------------------------------------------------
    wire [9:0] dy_bonus                    = bonus_y - 10'd120;
    wire signed [13:0] curve_shift_bonus   = curve_shift_at(curve_amount, dy_bonus);
    wire signed [13:0] center_x_bonus_signed = $signed({1'b0, ROAD_CENTER_X}) + curve_shift_bonus;
    wire [9:0] road_half_bonus             = ROAD_HALF_FAR + (dy_bonus >> 1);

    wire signed [10:0] bonus_lane_offset = (bonus_lane == 2'd0) ? -$signed({1'b0, road_half_bonus}) >>> 1 :
                                            (bonus_lane == 2'd2) ?  $signed({1'b0, road_half_bonus}) >>> 1 :
                                                                     11'sd0;

    wire signed [10:0] bonus_x_signed = center_x_bonus_signed + bonus_lane_offset;
    wire [9:0] bonus_x = clamp_center_x(bonus_x_signed, 10'd25);
    wire [9:0] bonus_r = BONUS_BASE_R + (dy_bonus >> BONUS_SCALE_SHIFT);

    // ------------------------------------------------------------------
    // Shape test: a small 4-point gold sparkle/star -- a tiny diamond
    // body with two tapering spikes along the axes (top/bottom,
    // left/right). The taper reuses the exact same cross-multiplication
    // "similar triangles" trick the curve warning arrow used to (no
    // division needed): at the center the spike is `thick` pixels wide,
    // shrinking linearly to a point at distance `r`. Deliberately a very
    // different silhouette from the obstacle's square hazard drum, so
    // the player can tell at a glance this one is good to touch.
    // Returns: 0 = none, 1 = star body/spike, 2 = bright core.
    // ------------------------------------------------------------------
    function [1:0] star_at;
        input [9:0] px, py;
        input [9:0] sx, sy;
        input [4:0] r;
        reg signed [10:0] ddx, ddy;
        reg [10:0] adx, ady;
        reg [10:0] thick, rr;
        reg [21:0] lhs_v, rhs_v, lhs_h, rhs_h; // widened products -- adx/ady can be up
                                                 // to ~640 (this runs for every pixel on
                                                 // screen, not just a local window around
                                                 // the star), and a plain 11-bit*11-bit
                                                 // self-determined multiply silently wraps
                                                 // around for inputs that large. That wrap
                                                 // could accidentally satisfy the <=
                                                 // comparison far from the real star,
                                                 // painting stray sparkle pixels wherever it
                                                 // happened to wrap low -- exactly the
                                                 // "gambar gajelas" glitch. 22 bits is more
                                                 // than enough (max product ~640*31 needs 15).
        reg core, body, vspike, hspike;
        begin
            ddx = $signed({1'b0, px}) - $signed({1'b0, sx});
            ddy = $signed({1'b0, py}) - $signed({1'b0, sy});
            adx = ddx[10] ? (-ddx) : ddx;
            ady = ddy[10] ? (-ddy) : ddy;
            rr    = {6'd0, r};
            thick = rr >> 2;
            if (thick < 11'd1) thick = 11'd1;

            core   = (adx + ady) <= (rr >> 2);
            body   = (adx + ady) <= (rr >> 1);

            lhs_v = {11'd0, adx}   * {11'd0, rr};
            rhs_v = {11'd0, thick} * {11'd0, (ady <= rr) ? (rr - ady) : 11'd0};
            lhs_h = {11'd0, ady}   * {11'd0, rr};
            rhs_h = {11'd0, thick} * {11'd0, (adx <= rr) ? (rr - adx) : 11'd0};

            vspike = (ady <= rr) && (lhs_v <= rhs_v);
            hspike = (adx <= rr) && (lhs_h <= rhs_h);

            if (core)
                star_at = 2'd2;
            else if (body || vspike || hspike)
                star_at = 2'd1;
            else
                star_at = 2'd0;
        end
    endfunction

    wire [1:0] bonus_shape  = bonus_active ? star_at(x, y, bonus_x, bonus_y, bonus_r[4:0]) : 2'd0;
    wire is_bonus_area = (bonus_shape != 2'd0);
    wire bonus_core    = (bonus_shape == 2'd2);
    wire bonus_body    = (bonus_shape == 2'd1);

    // ---- Collision box test (car vs obstacle), purely from registers
    // -- re-evaluated combinationally every time, but only actually
    // *sampled* once per frame below.
    wire signed [10:0] obs_left_s   = $signed({1'b0, obstacle_x}) - $signed({1'b0, obs_half_w});
    wire signed [10:0] obs_right_s  = $signed({1'b0, obstacle_x}) + $signed({1'b0, obs_half_w});
    wire signed [10:0] obs_top_s    = $signed({1'b0, obstacle_y}) - $signed({1'b0, obs_half_h});
    wire signed [10:0] obs_bottom_s = $signed({1'b0, obstacle_y}) + $signed({1'b0, obs_half_h});

    // ------------------------------------------------------------------
    // The car's SPRITE is a 128x128 box, but the actual visible car
    // photo inside it almost certainly doesn't fill that whole square
    // -- there's transparent/white padding above the roofline and
    // probably some on the sides too. Using the raw CAR_X/CAR_Y/CAR_W/
    // CAR_H box for collision makes the hitbox reach higher up the road
    // (toward the horizon) than the car actually appears, so it looks
    // like you get hit "from far away" when nothing is touching on
    // screen yet.
    //
    // Margin localparams (CAR_HITBOX_MARGIN_*) are declared earlier,
    // right next to CAR_STEP, so the dynamic road-boundary clamp above
    // can also use them.
    // ------------------------------------------------------------------
    wire signed [10:0] car_left_s   = $signed({1'b0, CAR_X}) + $signed({1'b0, CAR_HITBOX_MARGIN_SIDE});
    wire signed [10:0] car_right_s  = $signed({1'b0, CAR_X}) + $signed({1'b0, CAR_W}) - $signed({1'b0, CAR_HITBOX_MARGIN_SIDE});
    wire signed [10:0] car_top_s    = $signed({1'b0, CAR_Y}) + $signed({1'b0, CAR_HITBOX_MARGIN_TOP});
    wire signed [10:0] car_bottom_s = $signed({1'b0, CAR_Y}) + $signed({1'b0, CAR_H}) - $signed({1'b0, CAR_HITBOX_MARGIN_BOTTOM});

    wire obstacle_car_overlap = (obs_left_s   < car_right_s) && (obs_right_s  > car_left_s) &&
                                (obs_top_s    < car_bottom_s) && (obs_bottom_s > car_top_s);

    // Rising edge only -- obstacle_overlap_prev holds last frame's
    // overlap state, so collision_edge is true for exactly the one
    // frame where the boxes go from "not touching" to "touching".
    // Without this, `collision` would stay high for every one of the
    // ~15-50 frames the two boxes happen to overlap, which would look
    // like dozens of hits to whatever external module (score/lives)
    // is watching this signal for a single crash.
    wire collision_edge = obstacle_car_overlap && !obstacle_overlap_prev;

    // ------------------------------------------------------------------
    // Star pickup collision: same rectangular-box overlap + rising-edge
    // idiom as the obstacle above, just against bonus_x/bonus_y/bonus_r
    // instead -- touching the star (car hitbox overlaps its bounding
    // box) is a "catch", not a crash.
    // ------------------------------------------------------------------
    wire signed [10:0] bonus_left_s   = $signed({1'b0, bonus_x}) - $signed({1'b0, bonus_r});
    wire signed [10:0] bonus_right_s  = $signed({1'b0, bonus_x}) + $signed({1'b0, bonus_r});
    wire signed [10:0] bonus_top_s    = $signed({1'b0, bonus_y}) - $signed({1'b0, bonus_r});
    wire signed [10:0] bonus_bottom_s = $signed({1'b0, bonus_y}) + $signed({1'b0, bonus_r});

    wire bonus_car_overlap = bonus_active &&
                              (bonus_left_s   < car_right_s) && (bonus_right_s  > car_left_s) &&
                              (bonus_top_s    < car_bottom_s) && (bonus_bottom_s > car_top_s);

    wire bonus_collision_edge = bonus_car_overlap && !bonus_overlap_prev;
    wire bonus_reached_end    = bonus_active && ((bonus_y + OBSTACLE_SPEED) >= OBS_Y_END);

    // Points awarded each time an obstacle is successfully dodged
    // (reaches the end of the road without a collision this frame).
    localparam [15:0] POINTS_PER_DODGE = 16'd10;

    // ------------------------------------------------------------------
    // Combo bonus: consecutive successful dodges (no crash in between)
    // add extra points on top of POINTS_PER_DODGE, growing with the
    // streak up to a cap -- so chaining dodges together is worth more
    // than the same number of dodges spread across separate lives.
    // Resets to zero the instant a collision happens.
    // ------------------------------------------------------------------
    localparam [15:0] COMBO_BONUS_STEP = 16'd2;   // extra points added per combo level
    localparam [15:0] COMBO_BONUS_CAP  = 16'd50;  // max bonus any single dodge can add

    reg [7:0] combo_count;
    initial combo_count = 8'd0;

    wire [15:0] combo_bonus_raw = {8'd0, combo_count} * COMBO_BONUS_STEP;
    wire [15:0] combo_bonus     = (combo_bonus_raw > COMBO_BONUS_CAP) ? COMBO_BONUS_CAP : combo_bonus_raw;

    always @(posedge clk) begin
        if (reset) begin
            obstacle_y            <= OBS_Y_START;
            obstacle_lane         <= 2'd1;
            obstacle_is_car       <= 1'b1;
            obstacle_rand         <= 8'hA5;   // non-zero seed -- an all-zero LFSR would stay stuck at 0 forever
            obstacle_overlap_prev <= 1'b0;
            collision             <= 1'b0;
            score                 <= 16'd0;
            game_over              <= 1'b0;
            combo_count            <= 8'd0;
            lives                  <= LIVES_START;
            bonus_active           <= 1'b0;
            bonus_y                <= OBS_Y_START;
            bonus_lane             <= 2'd1;
            bonus_overlap_prev     <= 1'b0;
            bonus_spawn_timer      <= BONUS_SPAWN_INTERVAL;
        end else if (x == 10'd0 && y == 10'd0 && game_started) begin   // sekali per frame, tapi cuma setelah start ditekan
            // 8-bit maximal-length Fibonacci LFSR (taps 8,6,5,4).
            // Advancing this by a fixed number of steps each respawn
            // still lands on many different pseudo-random states,
            // unlike a plain "+1" counter which can freeze on one lane.
            obstacle_rand <= {obstacle_rand[6:0],
                               obstacle_rand[7] ^ obstacle_rand[5] ^ obstacle_rand[4] ^ obstacle_rand[3]};
            obstacle_overlap_prev <= obstacle_car_overlap;
            collision <= collision_edge;

            if (!game_over) begin
                // A crash costs one life AND some points (which also
                // quietly lowers the difficulty back down a notch,
                // since obstacle_speed_bonus is derived straight from
                // `score`) -- game_over only latches once lives hits 0.
                // Either way the combo streak breaks back to zero.
                if (collision_edge) begin
                    combo_count <= 8'd0;
                    score       <= (score >= LIFE_LOST_PENALTY) ? (score - LIFE_LOST_PENALTY) : 16'd0;

                    if (lives <= 3'd1) begin
                        lives     <= 3'd0;
                        game_over <= 1'b1;
                    end else begin
                        lives <= lives - 3'd1;
                    end
                end else if (obstacle_reached_end) begin
                    // Made it past this obstacle without hitting it --
                    // award points (base + current combo bonus) and
                    // extend the streak by one.
                    score       <= score + POINTS_PER_DODGE + combo_bonus;
                    combo_count <= combo_count + 8'd1;
                end

                // A fresh hit respawns the obstacle immediately (looks like
                // it "shattered" on impact, and guarantees the next frame's
                // overlap is false so we can't re-trigger on the same box).
                if (collision_edge || obstacle_reached_end) begin
                    obstacle_y            <= OBS_Y_START;
                    obstacle_lane         <= next_lane;
                    obstacle_is_car       <= next_is_car;
                    obstacle_overlap_prev <= 1'b0;
                end else begin
                    obstacle_y <= obstacle_y + OBSTACLE_SPEED;
                end

                // ------------------------------------------------------
                // Star pickup: independent of the obstacle above -- its
                // own spawn timer, own travel/lane, own overlap edge.
                // Catching it (bonus_collision_edge) grants +1 life (if
                // under the cap) and immediately retires the star;
                // missing it (bonus_reached_end) just retires it with no
                // penalty. Either way bonus_spawn_timer restarts the
                // countdown to the next star.
                // ------------------------------------------------------
                bonus_overlap_prev <= bonus_car_overlap;

                if (bonus_active) begin
                    if (bonus_collision_edge) begin
                        bonus_active       <= 1'b0;
                        bonus_spawn_timer  <= BONUS_SPAWN_INTERVAL;
                        if (lives < LIVES_MAX)
                            lives <= lives + 3'd1;
                    end else if (bonus_reached_end) begin
                        bonus_active      <= 1'b0;
                        bonus_spawn_timer <= BONUS_SPAWN_INTERVAL;
                    end else begin
                        bonus_y <= bonus_y + OBSTACLE_SPEED;
                    end
                end else begin
                    if (bonus_spawn_timer <= 16'd1) begin
                        bonus_active           <= 1'b1;
                        bonus_y                <= OBS_Y_START;
                        bonus_lane             <= next_lane; // reuses the obstacle LFSR's lane pick -- already decorrelated from the obstacle's own spawn timing
                        bonus_overlap_prev     <= 1'b0;
                    end else begin
                        bonus_spawn_timer <= bonus_spawn_timer - 16'd1;
                    end
                end
            end
            // if game_over is already high, obstacle/star/score/lives are
            // frozen in place until reset is asserted.
        end else if (x == 10'd0 && y == 10'd0) begin
            // Belum start_btn ditekan: obstacle_rand tetap dianggurkan,
            // tapi collision harus tetap di-clear supaya tidak nyangkut
            // tinggi kalau reset dilepas persis di tengah frame ganjil.
            collision <= 1'b0;
        end else begin
            collision <= 1'b0;   // cuma tinggi persis 1 clock tepat di frame tick tadi
        end
    end

    // ------------------------------------------------------------------
    // Screen fade: eases toward black while game_over is high (fade
    // OUT right after a crash) and eases back toward normal once
    // `reset` clears game_over and play resumes (fade IN) -- same
    // "step toward a target every frame" idiom already used for
    // curve_amount above. Deliberately its own tiny always block, not
    // folded into the reset/collision block above, so `reset` changes
    // the *target* (via game_over going low) rather than snapping
    // fade_level straight back to 0 -- that's what makes it fade IN
    // instead of just popping back to full brightness instantly.
    // ------------------------------------------------------------------
    localparam [3:0] FADE_MAX  = 4'd15;
    localparam [3:0] FADE_STEP = 4'd1;

    reg [3:0] fade_level;
    initial fade_level = 4'd0;

    wire [3:0] fade_target = game_over ? FADE_MAX : 4'd0;

    always @(posedge clk) begin
        if (x == 10'd0 && y == 10'd0) begin   // sekali per frame
            if (fade_level < fade_target)
                fade_level <= fade_level + FADE_STEP;
            else if (fade_level > fade_target)
                fade_level <= fade_level - FADE_STEP;
        end
    end

    // ------------------------------------------------------------------
    // High score: highest score reached so far. Deliberately NOT reset
    // by the `reset` signal (only its initial value at power-up is 0),
    // so it keeps a running "history" of the best run across as many
    // resets/restarts as the board stays powered on.
    // ------------------------------------------------------------------
    reg [15:0] high_score;
    initial high_score = 16'd0;

    always @(posedge clk) begin
        if (score > high_score)
            high_score <= score;
    end

    wire [23:0] car_rgb_straight, car_rgb_left, car_rgb_right;
    wire [23:0] car_rgb;

    // Instansiasi IP ROM -- sprite lurus (innova.mif, seperti semula)
    car_rom my_car_rom (
        .address (rom_addr),
        .clock   (clk),
        .q       (car_rgb_straight)
    );

    // Sprite serong kiri (car_left.mif -- hasil mirror dari serong kanan)
    car_rom_left my_car_rom_left (
        .address (rom_addr),
        .clock   (clk),
        .q       (car_rgb_left)
    );

    // Sprite serong kanan (car_right.mif)
    car_rom_right my_car_rom_right (
        .address (rom_addr),
        .clock   (clk),
        .q       (car_rgb_right)
    );

    // Pilih sprite mana yang tampil. Ketiga ROM tetap dibaca tiap
    // siklus (murah untuk block-RAM ROM di FPGA) -- yang dipilih di sini
    // cuma output mana yang dipakai untuk digambar.
    assign car_rgb = turn_left_active  ? car_rgb_left  :
                      turn_right_active ? car_rgb_right :
                                          car_rgb_straight;

    // ==================================================================
    // On-screen HUD: score digits (top-left), high-score digits (just
    // below it) and "GAME OVER" text (centered) drawn as a tiny
    // procedural 5x7 pixel font -- no external character ROM needed,
    // same style as the procedural obstacle/tree shapes above.
    // ==================================================================

    // ---- 5x7 font: returns a 5-bit row pattern (MSB = leftmost pixel)
    // for a given character code and row (0..6). chsel: 0-9 = digits,
    // 10=G 11=A 12=M 13=E 14=O 15=V 16=R 17=space 18=H 19=I.
    function [4:0] glyph_row;
        input [4:0] chsel;
        input [2:0] row;
        reg [4:0] r;
        begin
            case (chsel)
                5'd0: case(row) // '0'
                    3'd0: r=5'b01110; 3'd1: r=5'b10001; 3'd2: r=5'b10011; 3'd3: r=5'b10101;
                    3'd4: r=5'b11001; 3'd5: r=5'b10001; 3'd6: r=5'b01110; default: r=5'b00000;
                    endcase
                5'd1: case(row) // '1'
                    3'd0: r=5'b00100; 3'd1: r=5'b01100; 3'd2: r=5'b00100; 3'd3: r=5'b00100;
                    3'd4: r=5'b00100; 3'd5: r=5'b00100; 3'd6: r=5'b01110; default: r=5'b00000;
                    endcase
                5'd2: case(row) // '2'
                    3'd0: r=5'b01110; 3'd1: r=5'b10001; 3'd2: r=5'b00001; 3'd3: r=5'b00010;
                    3'd4: r=5'b00100; 3'd5: r=5'b01000; 3'd6: r=5'b11111; default: r=5'b00000;
                    endcase
                5'd3: case(row) // '3'
                    3'd0: r=5'b11111; 3'd1: r=5'b00010; 3'd2: r=5'b00100; 3'd3: r=5'b00010;
                    3'd4: r=5'b00001; 3'd5: r=5'b10001; 3'd6: r=5'b01110; default: r=5'b00000;
                    endcase
                5'd4: case(row) // '4'
                    3'd0: r=5'b00010; 3'd1: r=5'b00110; 3'd2: r=5'b01010; 3'd3: r=5'b10010;
                    3'd4: r=5'b11111; 3'd5: r=5'b00010; 3'd6: r=5'b00010; default: r=5'b00000;
                    endcase
                5'd5: case(row) // '5'
                    3'd0: r=5'b11111; 3'd1: r=5'b10000; 3'd2: r=5'b11110; 3'd3: r=5'b00001;
                    3'd4: r=5'b00001; 3'd5: r=5'b10001; 3'd6: r=5'b01110; default: r=5'b00000;
                    endcase
                5'd6: case(row) // '6'
                    3'd0: r=5'b00110; 3'd1: r=5'b01000; 3'd2: r=5'b10000; 3'd3: r=5'b11110;
                    3'd4: r=5'b10001; 3'd5: r=5'b10001; 3'd6: r=5'b01110; default: r=5'b00000;
                    endcase
                5'd7: case(row) // '7'
                    3'd0: r=5'b11111; 3'd1: r=5'b00001; 3'd2: r=5'b00010; 3'd3: r=5'b00100;
                    3'd4: r=5'b01000; 3'd5: r=5'b01000; 3'd6: r=5'b01000; default: r=5'b00000;
                    endcase
                5'd8: case(row) // '8'
                    3'd0: r=5'b01110; 3'd1: r=5'b10001; 3'd2: r=5'b10001; 3'd3: r=5'b01110;
                    3'd4: r=5'b10001; 3'd5: r=5'b10001; 3'd6: r=5'b01110; default: r=5'b00000;
                    endcase
                5'd9: case(row) // '9'
                    3'd0: r=5'b01110; 3'd1: r=5'b10001; 3'd2: r=5'b10001; 3'd3: r=5'b01111;
                    3'd4: r=5'b00001; 3'd5: r=5'b00010; 3'd6: r=5'b11100; default: r=5'b00000;
                    endcase
                5'd10: case(row) // 'G'
                    3'd0: r=5'b01110; 3'd1: r=5'b10001; 3'd2: r=5'b10000; 3'd3: r=5'b10111;
                    3'd4: r=5'b10001; 3'd5: r=5'b10001; 3'd6: r=5'b01111; default: r=5'b00000;
                    endcase
                5'd11: case(row) // 'A'
                    3'd0: r=5'b01110; 3'd1: r=5'b10001; 3'd2: r=5'b10001; 3'd3: r=5'b11111;
                    3'd4: r=5'b10001; 3'd5: r=5'b10001; 3'd6: r=5'b10001; default: r=5'b00000;
                    endcase
                5'd12: case(row) // 'M'
                    3'd0: r=5'b10001; 3'd1: r=5'b11011; 3'd2: r=5'b10101; 3'd3: r=5'b10101;
                    3'd4: r=5'b10001; 3'd5: r=5'b10001; 3'd6: r=5'b10001; default: r=5'b00000;
                    endcase
                5'd13: case(row) // 'E'
                    3'd0: r=5'b11111; 3'd1: r=5'b10000; 3'd2: r=5'b10000; 3'd3: r=5'b11110;
                    3'd4: r=5'b10000; 3'd5: r=5'b10000; 3'd6: r=5'b11111; default: r=5'b00000;
                    endcase
                5'd14: case(row) // 'O'
                    3'd0: r=5'b01110; 3'd1: r=5'b10001; 3'd2: r=5'b10001; 3'd3: r=5'b10001;
                    3'd4: r=5'b10001; 3'd5: r=5'b10001; 3'd6: r=5'b01110; default: r=5'b00000;
                    endcase
                5'd15: case(row) // 'V'
                    3'd0: r=5'b10001; 3'd1: r=5'b10001; 3'd2: r=5'b10001; 3'd3: r=5'b10001;
                    3'd4: r=5'b10001; 3'd5: r=5'b01010; 3'd6: r=5'b00100; default: r=5'b00000;
                    endcase
                5'd16: case(row) // 'R'
                    3'd0: r=5'b11110; 3'd1: r=5'b10001; 3'd2: r=5'b10001; 3'd3: r=5'b11110;
                    3'd4: r=5'b10100; 3'd5: r=5'b10010; 3'd6: r=5'b10001; default: r=5'b00000;
                    endcase
                5'd18: case(row) // 'H'
                    3'd0: r=5'b10001; 3'd1: r=5'b10001; 3'd2: r=5'b10001; 3'd3: r=5'b11111;
                    3'd4: r=5'b10001; 3'd5: r=5'b10001; 3'd6: r=5'b10001; default: r=5'b00000;
                    endcase
                5'd19: case(row) // 'I'
                    3'd0: r=5'b01110; 3'd1: r=5'b00100; 3'd2: r=5'b00100; 3'd3: r=5'b00100;
                    3'd4: r=5'b00100; 3'd5: r=5'b00100; 3'd6: r=5'b01110; default: r=5'b00000;
                    endcase
                5'd20: case(row) // 'S'
                    3'd0: r=5'b01111; 3'd1: r=5'b10000; 3'd2: r=5'b10000; 3'd3: r=5'b01110;
                    3'd4: r=5'b00001; 3'd5: r=5'b00001; 3'd6: r=5'b11110; default: r=5'b00000;
                    endcase
                5'd21: case(row) // 'T'
                    3'd0: r=5'b11111; 3'd1: r=5'b00100; 3'd2: r=5'b00100; 3'd3: r=5'b00100;
                    3'd4: r=5'b00100; 3'd5: r=5'b00100; 3'd6: r=5'b00100; default: r=5'b00000;
                    endcase
                default: r = 5'b00000; // 17 = space, or anything unmapped
            endcase
            glyph_row = r;
        end
    endfunction

    // Which glyph goes in each of the 9 "GAME OVER" character slots.
    function [4:0] gameover_char;
        input [3:0] idx;
        begin
            case (idx)
                4'd0: gameover_char = 5'd10; // G
                4'd1: gameover_char = 5'd11; // A
                4'd2: gameover_char = 5'd12; // M
                4'd3: gameover_char = 5'd13; // E
                4'd4: gameover_char = 5'd17; // space
                4'd5: gameover_char = 5'd14; // O
                4'd6: gameover_char = 5'd15; // V
                4'd7: gameover_char = 5'd13; // E
                4'd8: gameover_char = 5'd16; // R
                default: gameover_char = 5'd17;
            endcase
        end
    endfunction

    // Which glyph goes in each of the 5 "START" character slots.
    function [4:0] start_char;
        input [3:0] idx;
        begin
            case (idx)
                4'd0: start_char = 5'd20; // S
                4'd1: start_char = 5'd21; // T
                4'd2: start_char = 5'd11; // A
                4'd3: start_char = 5'd16; // R
                4'd4: start_char = 5'd21; // T
                default: start_char = 5'd17;
            endcase
        end
    endfunction

    // ---- Binary -> 5-digit BCD (double-dabble), so 16-bit score/
    // high-score registers can be split into individual decimal digits
    // to feed the font above. Pure combinational (loop is unrolled at
    // synthesis).
    function [19:0] bin_to_bcd;
        input [15:0] bin;
        integer i;
        reg [15:0] shifted;
        reg [19:0] bcd;
        begin
            bcd = 20'd0;
            shifted = bin;
            for (i = 0; i < 16; i = i + 1) begin
                if (bcd[3:0]   >= 5) bcd[3:0]   = bcd[3:0]   + 3;
                if (bcd[7:4]   >= 5) bcd[7:4]   = bcd[7:4]   + 3;
                if (bcd[11:8]  >= 5) bcd[11:8]  = bcd[11:8]  + 3;
                if (bcd[15:12] >= 5) bcd[15:12] = bcd[15:12] + 3;
                if (bcd[19:16] >= 5) bcd[19:16] = bcd[19:16] + 3;
                bcd     = {bcd[18:0], shifted[15]};
                shifted = {shifted[14:0], 1'b0};
            end
            bin_to_bcd = bcd;
        end
    endfunction

    wire [19:0] score_bcd      = bin_to_bcd(score);
    wire [19:0] high_score_bcd = bin_to_bcd(high_score);

    // ---- Score HUD position: top-left, 5 digits. Shrunk down (16px
    // cell / 2x glyph scale, was 32px cell / 4x scale) so the panel
    // takes up noticeably less screen real-estate while staying on a
    // power-of-2 cell size (16 = 2^4, scale 2 = 2^1) so every pixel
    // lookup below is still a free bit-slice, no dividers needed.
    localparam SCORE_X0 = 10'd20;
    localparam SCORE_Y0 = 10'd14;
    localparam SCORE_DIGITS = 4'd5;
    localparam SCORE_CELL = 10'd16; // 16px cell per digit (SCALE=2 glyph + padding)

    localparam SCORE_PANEL_MARGIN = 10'd6;
    localparam SCORE_BORDER_THICK = 10'd2;

    wire [9:0] score_box_w = SCORE_DIGITS * SCORE_CELL;
    wire [9:0] score_box_h = SCORE_CELL;

    wire in_score_box = (x >= SCORE_X0) && (x < SCORE_X0 + score_box_w) &&
                        (y >= SCORE_Y0) && (y < SCORE_Y0 + score_box_h);

    // Panel: a dark backdrop a bit bigger than the digits themselves,
    // with a bright gold border ring around the edge.
    wire in_score_panel = (x >= SCORE_X0 - SCORE_PANEL_MARGIN) && (x < SCORE_X0 + score_box_w + SCORE_PANEL_MARGIN) &&
                          (y >= SCORE_Y0 - SCORE_PANEL_MARGIN) && (y < SCORE_Y0 + score_box_h + SCORE_PANEL_MARGIN);

    wire in_score_border = in_score_panel &&
                           ((x < SCORE_X0 - SCORE_PANEL_MARGIN + SCORE_BORDER_THICK) ||
                            (x >= SCORE_X0 + score_box_w + SCORE_PANEL_MARGIN - SCORE_BORDER_THICK) ||
                            (y < SCORE_Y0 - SCORE_PANEL_MARGIN + SCORE_BORDER_THICK) ||
                            (y >= SCORE_Y0 + score_box_h + SCORE_PANEL_MARGIN - SCORE_BORDER_THICK));

    wire [9:0] score_xrel = x - SCORE_X0;
    wire [9:0] score_yrel = y - SCORE_Y0;
    wire [3:0] score_digit_idx = score_xrel[9:4];          // /16
    wire [2:0] score_col       = score_xrel[3:0] >> 1;      // within-cell col, /2 (SCALE)
    wire [2:0] score_row       = score_yrel[3:0] >> 1;      // within-cell row, /2 (SCALE)

    // Pick the BCD nibble for whichever digit column we're in.
    wire [3:0] score_digit_val = (score_digit_idx == 4'd0) ? score_bcd[19:16] :
                                  (score_digit_idx == 4'd1) ? score_bcd[15:12] :
                                  (score_digit_idx == 4'd2) ? score_bcd[11:8]  :
                                  (score_digit_idx == 4'd3) ? score_bcd[7:4]   :
                                                              score_bcd[3:0];

    wire [4:0] score_glyph_row = glyph_row({1'b0, score_digit_val}, score_row[2:0]);
    wire score_glyph_bit = in_score_box && (score_col < 3'd5) && (score_row < 3'd7) &&
                            score_glyph_row[4 - score_col];

    // ---- High-score HUD: label "HI" + 5 digits, same shrunk visual
    // style as the score panel, sat directly underneath it -- this is
    // the "history"/best-run readout, and it survives game resets
    // because high_score itself is never cleared by `reset`.
    localparam HI_LABEL_CHARS = 4'd2;
    localparam HI_DIGITS      = 4'd5;
    localparam HI_CELL        = 10'd16;
    localparam HI_X0 = SCORE_X0;
    localparam HI_Y0 = 10'd38; // just below the (now smaller) score panel

    wire [9:0] hi_box_w = (HI_LABEL_CHARS + HI_DIGITS) * HI_CELL;
    wire [9:0] hi_box_h = HI_CELL;

    wire in_hi_box = (x >= HI_X0) && (x < HI_X0 + hi_box_w) &&
                     (y >= HI_Y0) && (y < HI_Y0 + hi_box_h);

    wire in_hi_panel = (x >= HI_X0 - SCORE_PANEL_MARGIN) && (x < HI_X0 + hi_box_w + SCORE_PANEL_MARGIN) &&
                       (y >= HI_Y0 - SCORE_PANEL_MARGIN) && (y < HI_Y0 + hi_box_h + SCORE_PANEL_MARGIN);

    wire in_hi_border = in_hi_panel &&
                        ((x < HI_X0 - SCORE_PANEL_MARGIN + SCORE_BORDER_THICK) ||
                         (x >= HI_X0 + hi_box_w + SCORE_PANEL_MARGIN - SCORE_BORDER_THICK) ||
                         (y < HI_Y0 - SCORE_PANEL_MARGIN + SCORE_BORDER_THICK) ||
                         (y >= HI_Y0 + hi_box_h + SCORE_PANEL_MARGIN - SCORE_BORDER_THICK));

    wire [9:0] hi_xrel = x - HI_X0;
    wire [9:0] hi_yrel = y - HI_Y0;
    wire [3:0] hi_cell_idx = hi_xrel[9:4];        // /16, which of the 7 cells (2 label + 5 digits)
    wire [2:0] hi_col      = hi_xrel[3:0] >> 1;
    wire [2:0] hi_row      = hi_yrel[3:0] >> 1;

    // First two cells are the "H" and "I" label glyphs; the remaining
    // five are digits from high_score_bcd.
    wire [3:0] hi_digit_idx = hi_cell_idx - HI_LABEL_CHARS;
    wire [3:0] hi_digit_val = (hi_digit_idx == 4'd0) ? high_score_bcd[19:16] :
                               (hi_digit_idx == 4'd1) ? high_score_bcd[15:12] :
                               (hi_digit_idx == 4'd2) ? high_score_bcd[11:8]  :
                               (hi_digit_idx == 4'd3) ? high_score_bcd[7:4]   :
                                                          high_score_bcd[3:0];

    wire [4:0] hi_chsel = (hi_cell_idx == 4'd0) ? 5'd18 :        // 'H'
                          (hi_cell_idx == 4'd1) ? 5'd19 :        // 'I'
                                                    {1'b0, hi_digit_val};

    wire [4:0] hi_glyph_row = glyph_row(hi_chsel, hi_row);
    wire hi_glyph_bit = in_hi_box && (hi_col < 3'd5) && (hi_row < 3'd7) &&
                        hi_glyph_row[4 - hi_col];

    // ---- "GAME OVER" HUD position: centered, bigger (SCALE=4), 9
    // character cells padded into 32x32 so extraction is bit-slicing.
    localparam GO_X0 = 10'd176;
    localparam GO_Y0 = 10'd220;
    localparam GO_CHARS = 4'd9;

    wire in_go_box = game_over &&
                     (x >= GO_X0) && (x < GO_X0 + (GO_CHARS * 10'd32)) &&
                     (y >= GO_Y0) && (y < GO_Y0 + 10'd32);

    wire [9:0] go_xrel = x - GO_X0;
    wire [9:0] go_yrel = y - GO_Y0;
    wire [3:0] go_char_idx = go_xrel[9:5];             // /32
    wire [2:0] go_col      = go_xrel[4:0] >> 2;         // within-cell col, /4 (SCALE)
    wire [2:0] go_row      = go_yrel[4:0] >> 2;         // within-cell row, /4 (SCALE)

    wire [4:0] go_glyph_row = glyph_row(gameover_char(go_char_idx), go_row);
    wire go_glyph_bit = in_go_box && (go_col < 3'd5) && (go_row < 3'd7) &&
                        go_glyph_row[4 - go_col];

    // ---- "START" banner: same 32x32-cell style as GAME OVER, just 5
    // characters instead of 9, shown only before the player has
    // pressed start_btn for the first time.
    localparam ST_CHARS = 4'd5;
    localparam ST_X0 = (10'd640 - ST_CHARS * 10'd32) >> 1; // centered horizontally
    localparam ST_Y0 = GO_Y0;                                // same vertical spot GAME OVER uses

    wire in_st_box = !game_started &&
                     (x >= ST_X0) && (x < ST_X0 + (ST_CHARS * 10'd32)) &&
                     (y >= ST_Y0) && (y < ST_Y0 + 10'd32);

    wire [9:0] st_xrel = x - ST_X0;
    wire [9:0] st_yrel = y - ST_Y0;
    wire [3:0] st_char_idx = st_xrel[9:5];
    wire [2:0] st_col      = st_xrel[4:0] >> 2;
    wire [2:0] st_row      = st_yrel[4:0] >> 2;

    wire [4:0] st_glyph_row = glyph_row(start_char(st_char_idx), st_row);
    wire st_glyph_bit = in_st_box && (st_col < 3'd5) && (st_row < 3'd7) &&
                        st_glyph_row[4 - st_col];

    wire in_st_panel = !game_started &&
                       (x >= ST_X0 - SCORE_PANEL_MARGIN) && (x < ST_X0 + (ST_CHARS * 10'd32) + SCORE_PANEL_MARGIN) &&
                       (y >= ST_Y0 - SCORE_PANEL_MARGIN) && (y < ST_Y0 + 10'd32 + SCORE_PANEL_MARGIN);

    // ---- Lives panel: same shrunk visual family as the score/
    // high-score panels above, showing a single digit (0-5 now that
    // bonus lives can push past the original 3). Sits just below high
    // score.
    localparam LIVES_X0 = SCORE_X0;
    localparam LIVES_Y0 = HI_Y0 + HI_CELL + SCORE_PANEL_MARGIN * 2 + 10'd6; // stacked below the HI panel
    localparam LIVES_CELL = 10'd16;

    wire [9:0] lives_box_w = LIVES_CELL;
    wire [9:0] lives_box_h = LIVES_CELL;

    wire in_lives_box = (x >= LIVES_X0) && (x < LIVES_X0 + lives_box_w) &&
                        (y >= LIVES_Y0) && (y < LIVES_Y0 + lives_box_h);

    wire in_lives_panel = (x >= LIVES_X0 - SCORE_PANEL_MARGIN) && (x < LIVES_X0 + lives_box_w + SCORE_PANEL_MARGIN) &&
                          (y >= LIVES_Y0 - SCORE_PANEL_MARGIN) && (y < LIVES_Y0 + lives_box_h + SCORE_PANEL_MARGIN);

    wire in_lives_border = in_lives_panel &&
                           ((x < LIVES_X0 - SCORE_PANEL_MARGIN + SCORE_BORDER_THICK) ||
                            (x >= LIVES_X0 + lives_box_w + SCORE_PANEL_MARGIN - SCORE_BORDER_THICK) ||
                            (y < LIVES_Y0 - SCORE_PANEL_MARGIN + SCORE_BORDER_THICK) ||
                            (y >= LIVES_Y0 + lives_box_h + SCORE_PANEL_MARGIN - SCORE_BORDER_THICK));

    wire [9:0] lives_xrel = x - LIVES_X0;
    wire [9:0] lives_yrel = y - LIVES_Y0;
    wire [2:0] lives_col  = lives_xrel[3:0] >> 1;
    wire [2:0] lives_row  = lives_yrel[3:0] >> 1;

    wire [4:0] lives_glyph_row = glyph_row({2'd0, lives}, lives_row);
    wire lives_glyph_bit = in_lives_box && (lives_col < 3'd5) && (lives_row < 3'd7) &&
                           lives_glyph_row[4 - lives_col];

    // ------------------------------------------------------------------
    // Curve-direction HUD: a solid triangle/chevron pinned to the top
    // center of the screen, pointing whichever way the upcoming curve
    // bends -- replaces the old in-world roadside diamond sign, which
    // was easy to miss. Reuses sign_upcoming_left/right/sign_visible
    // (declared with the road-curve logic) for WHEN and WHICH way,
    // same triangle "similar triangles" cross-multiplication the old
    // sign's arrow used, just drawn as a fixed-position HUD element
    // instead of an object out in the 3D world.
    // ------------------------------------------------------------------
    function hud_arrow_at;
        input [9:0] px, py, cx, cy;
        input [4:0] half;
        input       dir;   // 0 = points left, 1 = points right
        reg signed [10:0] ddx, ddy, ady;
        reg signed [10:0] ddx_tri, r;
        begin
            ddx = $signed({1'b0, px}) - $signed({1'b0, cx});
            ddy = $signed({1'b0, py}) - $signed({1'b0, cy});
            ady = ddy[10] ? -ddy : ddy;
            r   = $signed({1'b0, half});
            ddx_tri = dir ? (ddx + r) : (r - ddx);
            hud_arrow_at = (ddx_tri >= 11'sd0) && (ddx_tri <= (r <<< 1)) &&
                           ((ady <<< 1) + ddx_tri <= (r <<< 1));
        end
    endfunction

    localparam [9:0] CURVE_HUD_W      = 10'd64;
    localparam [9:0] CURVE_HUD_H      = 10'd34;
    localparam [9:0] CURVE_HUD_X0     = (10'd640 - CURVE_HUD_W) >> 1; // centered horizontally
    localparam [9:0] CURVE_HUD_Y0     = 10'd6;
    localparam [9:0] CURVE_HUD_BORDER = 10'd2;

    wire in_curve_hud_panel = sign_visible &&
                              (x >= CURVE_HUD_X0) && (x < CURVE_HUD_X0 + CURVE_HUD_W) &&
                              (y >= CURVE_HUD_Y0) && (y < CURVE_HUD_Y0 + CURVE_HUD_H);

    wire in_curve_hud_border = in_curve_hud_panel &&
                              ((x < CURVE_HUD_X0 + CURVE_HUD_BORDER) ||
                               (x >= CURVE_HUD_X0 + CURVE_HUD_W - CURVE_HUD_BORDER) ||
                               (y < CURVE_HUD_Y0 + CURVE_HUD_BORDER) ||
                               (y >= CURVE_HUD_Y0 + CURVE_HUD_H - CURVE_HUD_BORDER));

    wire [9:0] curve_hud_cx = CURVE_HUD_X0 + (CURVE_HUD_W >> 1);
    wire [9:0] curve_hud_cy = CURVE_HUD_Y0 + (CURVE_HUD_H >> 1);

    wire curve_hud_arrow_pixel = sign_visible &&
                                  hud_arrow_at(x, y, curve_hud_cx, curve_hud_cy, 5'd12, sign_upcoming_right);

    reg [7:0] terrain_r, terrain_g, terrain_b;

    always @(*) begin
        if (!video_on) begin
            red   = 8'd0;
            green = 8'd0;
            blue  = 8'd0;
        end
 	else if (is_car_area && car_rgb != 24'hFFFFFF) begin
         red   = car_rgb[23:16];
         green = car_rgb[15:8];
         blue  = car_rgb[7:0];
     end

        else if (is_obstacle_area) begin
            if (obstacle_is_car) begin
                if (obs_car_window) begin
                    // Rear windshield -- dark glass.
                    red   = 8'd35;
                    green = 8'd40;
                    blue  = 8'd50;
                end else if (obs_car_taillight) begin
                    // Bright taillights.
                    red   = 8'd250;
                    green = 8'd40;
                    blue  = 8'd35;
                end else begin
                    // Car body.
                    red   = 8'd175;
                    green = 8'd35;
                    blue  = 8'd35;
                end
            end else begin
                if (obs_moto_helmet) begin
                    // Rider's helmet.
                    red   = 8'd25;
                    green = 8'd25;
                    blue  = 8'd30;
                end else if (obs_moto_taillight) begin
                    // Taillight.
                    red   = 8'd250;
                    green = 8'd40;
                    blue  = 8'd35;
                end else begin
                    // Body/seat.
                    red   = 8'd55;
                    green = 8'd60;
                    blue  = 8'd70;
                end
            end
        end

        else if (is_bonus_area) begin
            if (bonus_core) begin
                red   = 8'd255;
                green = 8'd255;
                blue  = 8'd230;   // bright near-white core
            end else begin
                red   = 8'd255;
                green = 8'd205;
                blue  = 8'd40;    // gold sparkle body
            end
        end

        else if (y < 10'd120) begin
			red   = {bg_pixel[11:8], bg_pixel[11:8]};
			green = {bg_pixel[7:4],  bg_pixel[7:4]};
			blue  = {bg_pixel[3:0],  bg_pixel[3:0]};
		  end

        else begin
            // ---- Figure out the flat terrain color first (same
            // priority chain as before), then blend it toward the
            // mountain/sky color sampled directly above this column
            // if we're still inside the horizon fade band. This is
            // what actually welds the ground into the background
            // image instead of hard-cutting at y==120.
            if (left_line_edge_aa || right_line_edge_aa) begin
                terrain_r = blend4(8'd65, 8'd255, line_edge_w);
                terrain_g = blend4(8'd65, 8'd255, line_edge_w);
                terrain_b = blend4(8'd65, 8'd255, line_edge_w);
            end

            else if (left_solid_line || right_solid_line || center_dash) begin
                terrain_r = 8'd255;
                terrain_g = 8'd255;
                terrain_b = 8'd255;
            end

            else if (road_area) begin
                // Dark purple-grey (blue channel highest) instead of
                // neutral grey, so the asphalt sits in the same color
                // family as the mountain silhouette.
                if (asphalt_fleck) begin
                    terrain_r = 8'd66;
                    terrain_g = 8'd62;
                    terrain_b = 8'd84;
                end else begin
                    terrain_r = 8'd56;
                    terrain_g = 8'd52;
                    terrain_b = 8'd76;
                end
            end

            else if (left_road_edge_aa || right_road_edge_aa) begin
                terrain_r = blend4(8'd140, 8'd56, road_edge_w);
                terrain_g = blend4(8'd116, 8'd52, road_edge_w);
                terrain_b = blend4(8'd148, 8'd76, road_edge_w);
            end

            else if (trotoar_left || trotoar_right) begin
                // Black-and-white striped kerb, like a real roadside curb.
                if (trotoar_stripe) begin
                    terrain_r = 8'd235;
                    terrain_g = 8'd235;
                    terrain_b = 8'd235;
                end else begin
                    terrain_r = 8'd18;
                    terrain_g = 8'd18;
                    terrain_b = 8'd18;
                end
            end

            else if (tree_canopy_outline) begin
                // Thin dark rim around the canopy so it reads as a
                // distinct rounded shape instead of a flat blob melting
                // into the sky/grass behind it.
                terrain_r = 8'd22;
                terrain_g = 8'd45;
                terrain_b = 8'd55;
            end

            else if (tree_canopy_lit) begin
                // Lit side (fixed "light from the left"): cooler
                // teal-green, brighter half of the canopy.
                terrain_r = 8'd62;
                terrain_g = 8'd118;
                terrain_b = 8'd130;
            end

            else if (tree_canopy_shadow) begin
                // Shadow side: same teal-green family, just darker --
                // this split is what actually gives the canopy some
                // roundness instead of looking like a flat disc.
                terrain_r = 8'd40;
                terrain_g = 8'd82;
                terrain_b = 8'd96;
            end

            else if (tree_trunk_lit) begin
                // Trunk lit side, mauve rather than plain brown to
                // echo the shoulder and mountain shadow tones.
                terrain_r = 8'd102;
                terrain_g = 8'd78;
                terrain_b = 8'd96;
            end

            else if (tree_trunk_shadow) begin
                // Trunk shadow side.
                terrain_r = 8'd72;
                terrain_g = 8'd54;
                terrain_b = 8'd68;
            end

            else if (pond_shore) begin
                // Muddy shore rim ringing the pond -- keeps the water
                // reading as a distinct irregular puddle shape instead
                // of melting straight into the grass.
                terrain_r = 8'd120;
                terrain_g = 8'd96;
                terrain_b = 8'd64;
            end

            else if (pond_water_lit) begin
                if (pond_shimmer) begin
                    // Bright drifting glint -- the water's only source
                    // of motion/life, since the shape itself is static.
                    terrain_r = 8'd170;
                    terrain_g = 8'd225;
                    terrain_b = 8'd235;
                end else begin
                    // Lit face (fixed "light from the left"): brighter
                    // reflective teal-blue.
                    terrain_r = 8'd70;
                    terrain_g = 8'd150;
                    terrain_b = 8'd180;
                end
            end

            else if (pond_water_shadow) begin
                // Shadow face: deeper, darker water -- gives the pond
                // some depth instead of a flat blob of blue.
                terrain_r = 8'd35;
                terrain_g = 8'd90;
                terrain_b = 8'd120;
            end

            else if (person_torso) begin
                // Shirt.
                terrain_r = 8'd60;
                terrain_g = 8'd90;
                terrain_b = 8'd150;
            end

            else if (person_legs) begin
                // Pants.
                terrain_r = 8'd55;
                terrain_g = 8'd55;
                terrain_b = 8'd62;
            end

            else if (person_arms) begin
                // Bare arms -- same skin tone as the head/face.
                terrain_r = 8'd210;
                terrain_g = 8'd165;
                terrain_b = 8'd130;
            end

            else if (person_head) begin
                // Face/skin.
                terrain_r = 8'd210;
                terrain_g = 8'd165;
                terrain_b = 8'd130;
            end

            else if (person_hair) begin
                // Dark hair cap.
                terrain_r = 8'd40;
                terrain_g = 8'd30;
                terrain_b = 8'd26;
            end

            else if (house_roof) begin
                // Dark brick-red roof.
                terrain_r = 8'd140;
                terrain_g = 8'd58;
                terrain_b = 8'd48;
            end

            else if (house_door) begin
                // Warm wooden door.
                terrain_r = 8'd96;
                terrain_g = 8'd64;
                terrain_b = 8'd44;
            end

            else if (house_window) begin
                // Glowing warm window light -- houses look inhabited.
                terrain_r = 8'd250;
                terrain_g = 8'd220;
                terrain_b = 8'd120;
            end

            else if (house_wall_lit) begin
                // Cream/adobe wall, lit side.
                terrain_r = 8'd222;
                terrain_g = 8'd196;
                terrain_b = 8'd160;
            end

            else if (house_wall_shadow) begin
                // Wall, shadow side.
                terrain_r = 8'd168;
                terrain_g = 8'd146;
                terrain_b = 8'd120;
            end

            else if (windmill_blade) begin
                // Bleached cream blades -- bright so the spin flip reads clearly.
                terrain_r = 8'd235;
                terrain_g = 8'd228;
                terrain_b = 8'd210;
            end

            else if (windmill_hub) begin
                // Small dark hub at the blade center.
                terrain_r = 8'd45;
                terrain_g = 8'd40;
                terrain_b = 8'd38;
            end

            else if (windmill_tower_lit) begin
                // Weathered stone tower, lit side.
                terrain_r = 8'd188;
                terrain_g = 8'd176;
                terrain_b = 8'd162;
            end

            else if (windmill_tower_shadow) begin
                // Tower, shadow side.
                terrain_r = 8'd132;
                terrain_g = 8'd122;
                terrain_b = 8'd112;
            end

            else begin
                // Grass: 4 close, subtle teal-green shades picked
                // per-pixel by grass_speck_a/b, instead of one flat
                // block of color -- breaks up the "solid green carpet"
                // look without changing the overall dusk-meadow hue.
                case ({grass_speck_a, grass_speck_b})
                    2'b00: begin terrain_r = 8'd52; terrain_g = 8'd104; terrain_b = 8'd116; end
                    2'b01: begin terrain_r = 8'd58; terrain_g = 8'd112; terrain_b = 8'd124; end
                    2'b10: begin terrain_r = 8'd64; terrain_g = 8'd120; terrain_b = 8'd132; end
                    default: begin terrain_r = 8'd70; terrain_g = 8'd128; terrain_b = 8'd140; end
                endcase
            end

            // ---- Horizon fade: for the first few rows below the
            // background image, ease from the exact mountain/sky color
            // above this column into the flat terrain color computed
            // above. Everything further down the screen is untouched.
            if (in_horizon_blend) begin
                red   = blend4(horizon_r, terrain_r, horizon_blend_w);
                green = blend4(horizon_g, terrain_g, horizon_blend_w);
                blue  = blend4(horizon_b, terrain_b, horizon_blend_w);
            end else begin
                red   = terrain_r;
                green = terrain_g;
                blue  = terrain_b;
            end
        end

        // ---- Screen fade: applied to the scene we just computed
        // above, but BEFORE the HUD block below -- so the fade darkens
        // the road/car/background, while score/HUD/GAME OVER text
        // (drawn next) stay fully readable even at full fade.
        red   = blend4(red,   8'd0, fade_level);
        green = blend4(green, 8'd0, fade_level);
        blue  = blend4(blue,  8'd0, fade_level);

        // ---- HUD overlay: drawn last so it always sits on top of
        // everything else (road, car, obstacles, background).
        if (video_on) begin
            // Panel first, so the digits drawn afterward sit on top of it.
            if (in_score_panel) begin
                if (in_score_border) begin
                    red   = 8'd255;
                    green = 8'd200;
                    blue  = 8'd60;   // bright gold border ring
                end else begin
                    red   = 8'd18;
                    green = 8'd16;
                    blue  = 8'd30;   // dark navy backdrop so digits pop
                end
            end
            if (score_glyph_bit) begin
                red   = 8'd255;
                green = 8'd230;
                blue  = 8'd40;   // yellow score digits, easy to spot
            end

            if (in_hi_panel) begin
                if (in_hi_border) begin
                    red   = 8'd200;
                    green = 8'd200;
                    blue  = 8'd220;   // silver border ring for the high-score panel
                end else begin
                    red   = 8'd18;
                    green = 8'd16;
                    blue  = 8'd30;
                end
            end
            if (hi_glyph_bit) begin
                red   = 8'd200;
                green = 8'd220;
                blue  = 8'd255;   // pale blue high-score digits
            end

            if (in_lives_panel) begin
                if (in_lives_border) begin
                    red   = 8'd255;
                    green = 8'd120;
                    blue  = 8'd120;   // soft red border ring for the lives panel
                end else begin
                    red   = 8'd18;
                    green = 8'd16;
                    blue  = 8'd30;
                end
            end
            if (lives_glyph_bit) begin
                red   = 8'd255;
                green = 8'd100;
                blue  = 8'd100;   // red-ish "lives" digit
            end

            if (in_curve_hud_panel) begin
                if (in_curve_hud_border) begin
                    red   = 8'd255;
                    green = 8'd190;
                    blue  = 8'd40;    // amber warning border, same family as the old sign
                end else begin
                    red   = 8'd18;
                    green = 8'd16;
                    blue  = 8'd30;
                end
            end
            if (curve_hud_arrow_pixel) begin
                red   = 8'd255;
                green = 8'd225;
                blue  = 8'd60;    // bright yellow chevron
            end

            if (in_st_panel) begin
                red   = 8'd18;
                green = 8'd16;
                blue  = 8'd30;
            end
            if (st_glyph_bit) begin
                red   = 8'd120;
                green = 8'd255;
                blue  = 8'd140;   // green "START" text -- distinct from the red GAME OVER
            end

            if (go_glyph_bit) begin
                red   = 8'd255;
                green = 8'd40;
                blue  = 8'd40;   // red "GAME OVER" text
            end
        end
    end

endmodule
