module render_static ( 
    input  wire       clk,
    input  wire       video_on,
	 input  wire 		 reset,
    input  wire [9:0] x,
    input  wire [9:0] y,
    input  wire       steer_left,   // active-high: hold to move the car left
    input  wire       steer_right,  // active-high: hold to move the car right
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

    wire [9:0] dy;
    wire [9:0] road_left;
    wire [9:0] road_right;
    wire [9:0] road_width;
    wire [9:0] line_margin;
    wire [9:0] line_thick;

    assign dy = (y >= 10'd120) ? (y - 10'd120) : 10'd0;

    // How fast the world scrolls toward the camera each frame -- used by
    // both the road curve timing below and the dashed-line/tree motion
    // further down.
    localparam SCROLL_SPEED = 10'd3;

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
    // curve_base runs from 45 at the horizon (dy=0) down to 0 at the
    // very bottom of the screen (dy=360, i.e. y=480) -- dy>>3 is just
    // dy/8, and 360/8=45, so it lands on exactly 0 with no division.
    //
    // This is fully automatic and cycles through 4 phases, each lasting
    // SEGMENT_LENGTH pixels of travel: straight -> curve right ->
    // straight -> curve left -> repeat. curve_amount eases toward
    // whatever the current phase's target is by CURVE_STEP per frame,
    // so transitions are smooth instead of an instant kink.
    // ------------------------------------------------------------------
    // CURVE_MAX_CAP=6 is the safe ceiling: past this, curved_center_x
    // would push road_left below 0 at the horizon, which underflows the
    // unsigned subtraction and makes the road glitch/wrap on screen.
    localparam signed CURVE_MAX_CAP       = 6;
    localparam signed CURVE_MAX_START     = 1;   // how sharp the very first bend is
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
    reg signed [4:0]  curve_amount;  // -current_curve_max .. +current_curve_max
    reg [7:0]         curve_update_counter;
    reg signed [4:0]  current_curve_max;  // grows by CURVE_MAX_INCREMENT each full loop, capped at CURVE_MAX_CAP

    initial current_curve_max = CURVE_MAX_START;

    wire signed [4:0] target_curve = (phase == PHASE_RIGHT) ? current_curve_max :
                                      (phase == PHASE_LEFT)  ? -current_curve_max :
                                                                5'sd0;

    wire [9:0] curve_base = 10'd45 - (dy >> 3);

    always @(posedge clk) begin
        if (reset) begin
            seg_dist             <= 20'd0;
            phase                <= PHASE_STRAIGHT_1;
            curve_amount         <= 5'sd0;
            curve_update_counter <= 8'd0;
            current_curve_max    <= CURVE_MAX_START;
        end else if (x == 10'd0 && y == 10'd0 && !game_over) begin   // once per frame, freeze on game over
            if (seg_dist + SCROLL_SPEED >= SEGMENT_LENGTH) begin
                seg_dist <= (seg_dist + SCROLL_SPEED) - SEGMENT_LENGTH;

                // Completing PHASE_LEFT means the next phase wraps back
                // to PHASE_STRAIGHT_1 -- i.e. a full loop just finished.
                // Make the next loop's bends a bit sharper, up to the cap.
                if (phase == PHASE_LEFT && current_curve_max < CURVE_MAX_CAP)
                    current_curve_max <= current_curve_max + CURVE_MAX_INCREMENT;

                phase <= phase + 2'd1;      // wraps 0,1,2,3,0,... automatically
            end else begin
                seg_dist <= seg_dist + SCROLL_SPEED;
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

    wire signed [10:0] curve_shift;
    assign curve_shift = curve_amount * $signed({1'b0, curve_base});

    wire signed [10:0] center_x_signed;
    assign center_x_signed = $signed({1'b0, ROAD_CENTER_X}) + curve_shift;

    wire [9:0] curved_center_x;
    assign curved_center_x = center_x_signed[9:0];

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
    // SCROLL_SPEED (declared earlier, next to the curve timer) controls
    // how fast the road appears to move: bigger = faster.
    reg [9:0] scroll_y;

    always @(posedge clk) begin
        if (reset)
            scroll_y <= 10'd0;
        else if (x == 10'd0 && y == 10'd0 && !game_over)
            scroll_y <= scroll_y + SCROLL_SPEED;
    end

    // NOTE: subtract, not add. Sampling the pattern at (y - scroll_y)
    // makes a fixed dash/fleck boundary drift to LARGER y as scroll_y
    // grows -- i.e. down the screen, toward the camera -- which reads
    // as forward motion. Adding scroll_y instead makes it drift the
    // other way (up toward the horizon), which looks like reversing.
    wire [9:0] anim_y = y - scroll_y;

    assign asphalt_fleck = x[1] ^ x[4] ^ anim_y[2] ^ anim_y[5];

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
        reg [9:0] offset;
        begin
            offset  = (ty - Y_MIN + scroll) % RANGE;
            wrap_ty = Y_MIN + offset;
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

            wire [9:0] this_cbase = 10'd45 - (this_dy >> 3);
            wire signed [10:0] this_cshift  = curve_amount * $signed({1'b0, this_cbase});
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

            wire [9:0] this_cbase = 10'd45 - (this_dy >> 3);
            wire signed [10:0] this_cshift  = curve_amount * $signed({1'b0, this_cbase});
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
    wire [9:0] curve_base_car = 10'd45 - (dy_car >> 3);
    wire signed [10:0] curve_shift_car     = curve_amount * $signed({1'b0, curve_base_car});
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

    always @(posedge clk) begin
        if (x == 10'd0 && y == 10'd0) begin   // sekali per frame
            if (steer_left && !steer_right) begin
                if ((car_offset - CAR_STEP) < car_offset_min_eff)
                    car_offset <= car_offset_min_eff;
                else
                    car_offset <= car_offset - CAR_STEP;
            end else if (steer_right && !steer_left) begin
                if ((car_offset + CAR_STEP) > car_offset_max_eff)
                    car_offset <= car_offset_max_eff;
                else
                    car_offset <= car_offset + CAR_STEP;
            end else begin
                // Tidak ada tombol ditahan -> diam di tempat, tapi tetap
                // dijepit ulang kalau-kalau lengkungan jalan baru saja
                // menyempit di bawah posisi mobil yang sedang diam.
                if (car_offset < car_offset_min_eff)
                    car_offset <= car_offset_min_eff;
                else if (car_offset > car_offset_max_eff)
                    car_offset <= car_offset_max_eff;
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

    wire [15:0] speed_tier_raw = score / OBSTACLE_SPEEDUP_SCORE;
    wire [9:0]  obstacle_speed_bonus = (speed_tier_raw > {6'd0, OBSTACLE_SPEED_MAX_BONUS}) ?
                                        OBSTACLE_SPEED_MAX_BONUS : speed_tier_raw[9:0];
    wire [9:0] OBSTACLE_SPEED = SCROLL_SPEED + obstacle_speed_bonus; // travels at road speed + difficulty bonus

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
    wire [9:0] curve_base_obs = 10'd45 - (dy_obs >> 3);
    wire signed [10:0] curve_shift_obs = curve_amount * $signed({1'b0, curve_base_obs});
    wire signed [10:0] center_x_obs_signed = $signed({1'b0, ROAD_CENTER_X}) + curve_shift_obs;
    wire [9:0] road_half_obs  = ROAD_HALF_FAR + (dy_obs >> 1);

    // Lane offset as a fraction of the road's current half-width at
    // this distance, so the obstacle stays inside the road at any
    // point along its approach instead of drifting into the grass.
    wire signed [10:0] lane_offset = (obstacle_lane == 2'd0) ? -$signed({1'b0, road_half_obs}) >>> 1 :
                                      (obstacle_lane == 2'd2) ?  $signed({1'b0, road_half_obs}) >>> 1 :
                                                                  11'sd0;

    wire signed [10:0] obstacle_x_signed = center_x_obs_signed + lane_offset;
    wire [9:0] obstacle_x = obstacle_x_signed[9:0];

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
        end else if (x == 10'd0 && y == 10'd0) begin   // sekali per frame
            // 8-bit maximal-length Fibonacci LFSR (taps 8,6,5,4).
            // Advancing this by a fixed number of steps each respawn
            // still lands on many different pseudo-random states,
            // unlike a plain "+1" counter which can freeze on one lane.
            obstacle_rand <= {obstacle_rand[6:0],
                               obstacle_rand[7] ^ obstacle_rand[5] ^ obstacle_rand[4] ^ obstacle_rand[3]};
            obstacle_overlap_prev <= obstacle_car_overlap;
            collision <= collision_edge;

            if (!game_over) begin
                // A crash ends the game immediately -- game_over stays
                // high until the next reset. A crash also breaks the
                // combo streak back down to zero.
                if (collision_edge) begin
                    game_over   <= 1'b1;
                    combo_count <= 8'd0;
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
            // if game_over is already high, obstacle/score are frozen
            // in place until reset is asserted.
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
                // Grass: desaturated teal-green with blue pulled up
                // close to green, instead of a saturated summer green
                // -- reads as dusk-lit meadow, same family as the sky.
                terrain_r = 8'd58;
                terrain_g = 8'd112;
                terrain_b = 8'd124;
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

            if (go_glyph_bit) begin
                red   = 8'd255;
                green = 8'd40;
                blue  = 8'd40;   // red "GAME OVER" text
            end
        end
    end

endmodule
