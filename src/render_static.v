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
        input signed [10:0] cx;
        input [9:0] half_extent;
        reg signed [10:0] lo, hi;
        begin
            lo = $signed({1'b0, half_extent});
            hi = 11'sd639 - $signed({1'b0, half_extent});
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
    function signed [10:0] curve_shift_at;
        input signed [6:0] camt;   // curve_amount (or a snapshot of it)
        input        [9:0] dyv;    // distance from horizon for this row/object
        reg          [9:0] cbase_fine;  // 360 (horizon) .. 0 (bottom of screen)
        reg signed  [17:0] prod;
        begin
            cbase_fine     = 10'd360 - dyv;
            prod           = camt * $signed({1'b0, cbase_fine});
            curve_shift_at = prod >>> 3;   // same overall /8 scale the old code used
        end
    endfunction

    wire signed [10:0] curve_shift;
    assign curve_shift = curve_shift_at(curve_amount, dy);

    wire signed [10:0] center_x_signed;
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

    // Dirt/gravel shoulder: a narrow strip just outside the asphalt on
    // both sides, so the road doesn't cut straight to grass.
    localparam SHOULDER_WIDTH = 6;
    wire shoulder_left, shoulder_right;

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
    // Treeline: instead of a handful of trees at fixed spots, generate
    // NUM_TREES_SIDE evenly-spaced trees per side that each cycle
    // through the FULL horizon-to-camera range (via wrap_ty), scale up
    // as they approach (same growth idea as the obstacle), and hug
    // whichever way the road is currently curving -- so the treeline
    // reads as one continuous row from one end of the road to the
    // other, on both sides, at every point in time.
    // ------------------------------------------------------------------
    localparam NUM_TREES_SIDE      = 8;
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

            wire signed [10:0] this_cshift  = curve_shift_at(curve_amount, this_dy);
            wire signed [10:0] this_censig  = $signed({1'b0, ROAD_CENTER_X}) + this_cshift;
            wire [9:0] this_center = this_censig[9:0];

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

            wire signed [10:0] this_cshift  = curve_shift_at(curve_amount, this_dy);
            wire signed [10:0] this_censig  = $signed({1'b0, ROAD_CENTER_X}) + this_cshift;
            wire [9:0] this_center = this_censig[9:0];

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
   
    assign shoulder_left  = (y >= 10'd120) &&
                             (x >= road_left - SHOULDER_WIDTH) &&
                             (x <  road_left);

    assign shoulder_right = (y >= 10'd120) &&
                             (x >  road_right) &&
                             (x <= road_right + SHOULDER_WIDTH);

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
    wire signed [10:0] curve_shift_car     = curve_shift_at(curve_amount, dy_car);
    wire signed [10:0] center_x_car_signed = $signed({1'b0, ROAD_CENTER_X}) + curve_shift_car;
    wire [9:0] curved_center_x_car = center_x_car_signed[9:0];
    wire [9:0] road_half_car  = ROAD_HALF_FAR + (dy_car >> 1);
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
    // Obstacle (rintangan): a single procedural "barrier drum" that
    // travels from just below the horizon down to the player's row,
    // scaling up the same way the road itself widens with dy, then
    // loops back to the horizon with a new random lane. Drawn as a
    // colored box (same style family as the trees) instead of a photo
    // sprite, so it needs no new ROM/image asset.
    // ------------------------------------------------------------------
    localparam OBS_Y_START      = 10'd130;          // spawn row, just past the horizon
    localparam OBS_Y_END        = CAR_Y + CAR_H;     // once it reaches here it has "passed" the player
    // Diperkecil sedikit dari sebelumnya (6/6, shift 3) supaya rintangan
    // tidak terlalu besar/menakutkan dan tumbuh lebih halus saat mendekat.
    localparam OBS_BASE_HALF_W  = 10'd3;   // half-width at the horizon (small/far away)
    localparam OBS_BASE_HALF_H  = 10'd3;   // half-height at the horizon
    localparam OBS_SCALE_SHIFT  = 5;       // how fast it grows with distance (smaller = grows faster)

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
    reg [7:0] obstacle_rand;   // 8-bit LFSR, sampled at respawn for a pseudo-random lane
    reg       obstacle_overlap_prev; // overlap state one frame ago, for edge detection

    // ------------------------------------------------------------------
    // Lives: 3 chances instead of instant game over. Each hit costs one
    // life AND LIFE_LOST_PENALTY points (which also naturally lowers
    // obstacle_speed_bonus/difficulty since that's derived straight
    // from `score` -- no separate "slow back down" logic needed, it
    // falls out of the existing difficulty formula for free). Only the
    // hit that takes the LAST life actually sets game_over.
    // ------------------------------------------------------------------
    localparam [1:0]  LIVES_START       = 2'd3;
    localparam [15:0] LIFE_LOST_PENALTY = 16'd10;

    reg [1:0] lives;
    initial lives = LIVES_START;

    initial obstacle_y            = OBS_Y_START;
    initial obstacle_lane         = 2'd1;
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

    // Same perspective math the road itself uses (curve_base/curved
    // center/half-width), just evaluated at the obstacle's OWN distance
    // (dy_obs) instead of the current scanline's dy -- it's a flat box,
    // so one size/position per frame is enough, no per-row recompute.
    wire [9:0] dy_obs         = obstacle_y - 10'd120;
    wire signed [10:0] curve_shift_obs = curve_shift_at(curve_amount, dy_obs);
    wire signed [10:0] center_x_obs_signed = $signed({1'b0, ROAD_CENTER_X}) + curve_shift_obs;
    wire [9:0] road_half_obs  = ROAD_HALF_FAR + (dy_obs >> 1);

    // Lane offset as a fraction of the road's current half-width at
    // this distance, so the obstacle stays inside the road at any
    // point along its approach instead of drifting into the grass.
    wire signed [10:0] lane_offset = (obstacle_lane == 2'd0) ? -$signed({1'b0, road_half_obs}) >>> 1 :
                                      (obstacle_lane == 2'd2) ?  $signed({1'b0, road_half_obs}) >>> 1 :
                                                                  11'sd0;

    wire signed [10:0] obstacle_x_signed = center_x_obs_signed + lane_offset;
    wire [9:0] obstacle_x = clamp_center_x(obstacle_x_signed, 10'd25); // 25 > OBS_MAX_HALF_W, safe margin

    wire [9:0] obs_half_w = OBS_BASE_HALF_W + (dy_obs >> OBS_SCALE_SHIFT);
    wire [9:0] obs_half_h = OBS_BASE_HALF_H + (dy_obs >> OBS_SCALE_SHIFT);

    wire signed [10:0] obs_col_rel = $signed({1'b0, x}) - $signed({1'b0, obstacle_x});
    wire signed [10:0] obs_row_rel = $signed({1'b0, y}) - $signed({1'b0, obstacle_y});

    wire is_obstacle_area = (obs_col_rel >= -$signed({1'b0, obs_half_w})) && (obs_col_rel <= $signed({1'b0, obs_half_w})) &&
                             (obs_row_rel >= -$signed({1'b0, obs_half_h})) && (obs_row_rel <= $signed({1'b0, obs_half_h}));

    // Warning-drum look: dark border, a black stripe through the
    // middle third, orange everywhere else -- reads clearly as "hazard"
    // regardless of the scene's color palette.
    wire [9:0] obs_border_px  = 10'd2;
    wire obs_is_border = (obs_col_rel <= -$signed({1'b0, obs_half_w}) + $signed({1'b0, obs_border_px})) ||
                          (obs_col_rel >=  $signed({1'b0, obs_half_w}) - $signed({1'b0, obs_border_px})) ||
                          (obs_row_rel <= -$signed({1'b0, obs_half_h}) + $signed({1'b0, obs_border_px})) ||
                          (obs_row_rel >=  $signed({1'b0, obs_half_h}) - $signed({1'b0, obs_border_px}));

    wire [9:0] obs_mid_band = obs_half_h >> 2;
    wire obs_is_stripe = (obs_row_rel >= -$signed({1'b0, obs_mid_band})) && (obs_row_rel <= $signed({1'b0, obs_mid_band}));

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
            obstacle_rand         <= 8'hA5;   // non-zero seed -- an all-zero LFSR would stay stuck at 0 forever
            obstacle_overlap_prev <= 1'b0;
            collision             <= 1'b0;
            score                 <= 16'd0;
            game_over              <= 1'b0;
            combo_count            <= 8'd0;
            lives                  <= LIVES_START;
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

                    if (lives <= 2'd1) begin
                        lives     <= 2'd0;
                        game_over <= 1'b1;
                    end else begin
                        lives <= lives - 2'd1;
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
                    obstacle_overlap_prev <= 1'b0;
                end else begin
                    obstacle_y <= obstacle_y + OBSTACLE_SPEED;
                end
            end
            // if game_over is already high, obstacle/score/lives are
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

    // ---- Score HUD position: top-left, 5 digits, scaled up to match
    // the GAME OVER text (SCALE=4) so it's actually easy to read, sat
    // on top of a dark panel with a bright border so it stands out
    // against the sky/mountain background behind it instead of
    // blending in.
    localparam SCORE_X0 = 10'd20;
    localparam SCORE_Y0 = 10'd16;
    localparam SCORE_DIGITS = 4'd5;
    localparam SCORE_CELL = 10'd32; // 32px cell per digit (SCALE=4 glyph + padding)

    localparam SCORE_PANEL_MARGIN = 10'd10;
    localparam SCORE_BORDER_THICK = 10'd3;

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
    wire [3:0] score_digit_idx = score_xrel[9:5];          // /32
    wire [2:0] score_col       = score_xrel[4:0] >> 2;      // within-cell col, /4 (SCALE)
    wire [2:0] score_row       = score_yrel[4:0] >> 2;      // within-cell row, /4 (SCALE)

    // Pick the BCD nibble for whichever digit column we're in.
    wire [3:0] score_digit_val = (score_digit_idx == 4'd0) ? score_bcd[19:16] :
                                  (score_digit_idx == 4'd1) ? score_bcd[15:12] :
                                  (score_digit_idx == 4'd2) ? score_bcd[11:8]  :
                                  (score_digit_idx == 4'd3) ? score_bcd[7:4]   :
                                                              score_bcd[3:0];

    wire [4:0] score_glyph_row = glyph_row({1'b0, score_digit_val}, score_row[2:0]);
    wire score_glyph_bit = in_score_box && (score_col < 3'd5) && (score_row < 3'd7) &&
                            score_glyph_row[4 - score_col];

    // ---- High-score HUD: label "HI" + 5 digits, same visual style as
    // the score panel, sat directly underneath it -- this is the
    // "history"/best-run readout, and it survives game resets because
    // high_score itself is never cleared by `reset`.
    localparam HI_LABEL_CHARS = 4'd2;
    localparam HI_DIGITS      = 4'd5;
    localparam HI_CELL        = 10'd32;
    localparam HI_X0 = SCORE_X0;
    localparam HI_Y0 = 10'd70; // just below the score panel (which ends around y=58)

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
    wire [3:0] hi_cell_idx = hi_xrel[9:5];        // /32, which of the 7 cells (2 label + 5 digits)
    wire [2:0] hi_col      = hi_xrel[4:0] >> 2;
    wire [2:0] hi_row      = hi_yrel[4:0] >> 2;

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

    // ---- Lives panel: same visual family as the score/high-score
    // panels, showing a single digit (0-3). Sits just below high score.
    localparam LIVES_X0 = SCORE_X0;
    localparam LIVES_Y0 = HI_Y0 + HI_CELL + SCORE_PANEL_MARGIN * 2 + 10'd10; // stacked below the HI panel
    localparam LIVES_CELL = 10'd32;

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
    wire [2:0] lives_col  = lives_xrel[4:0] >> 2;
    wire [2:0] lives_row  = lives_yrel[4:0] >> 2;

    wire [4:0] lives_glyph_row = glyph_row({3'd0, lives}, lives_row);
    wire lives_glyph_bit = in_lives_box && (lives_col < 3'd5) && (lives_row < 3'd7) &&
                           lives_glyph_row[4 - lives_col];

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
            if (obs_is_border) begin
                red   = 8'd35;
                green = 8'd25;
                blue  = 8'd20;
            end else if (obs_is_stripe) begin
                red   = 8'd20;
                green = 8'd20;
                blue  = 8'd20;
            end else begin
                red   = 8'd235;
                green = 8'd120;
                blue  = 8'd35;
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

            else if (shoulder_left || shoulder_right) begin
                // Dusty mauve gravel, pulled toward the mountains' blue
                // undertone rather than a plain warm brown.
                terrain_r = 8'd140;
                terrain_g = 8'd116;
                terrain_b = 8'd148;
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
