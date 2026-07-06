module render_static ( 
    input  wire       clk,
    input  wire       video_on,
    input  wire [9:0] x,
    input  wire [9:0] y,
    input  wire       steer_left,   // active-high: hold to move the car left
    input  wire       steer_right,  // active-high: hold to move the car right
    output reg  [7:0] red,
    output reg  [7:0] green,
    output reg  [7:0] blue
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
        if (x == 10'd0 && y == 10'd0) begin   // once per frame
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
        if (x == 10'd0 && y == 10'd0)
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
    // Returns: 2'b01 = inside canopy, 2'b10 = inside trunk, 2'b00 = none
    // ------------------------------------------------------------------
    function [1:0] tree_at;
        input [9:0] px, py;   // pixel under test
        input [9:0] tx, ty;   // tree position: x center, y = ground/base
        input [4:0] cr;       // canopy radius
        input [4:0] tw, th;   // trunk width, trunk height
        reg   [9:0] cy;       // canopy center y
        reg   [9:0] dx, dyv;
        reg   [19:0] dist2, r2;
        begin
            cy   = ty - th - cr + 10'd3;  // canopy overlaps top of trunk a bit
            dx   = (px >= tx) ? (px - tx) : (tx - px);
            dyv  = (py >= cy) ? (py - cy) : (cy - py);
            dist2 = dx * dx + dyv * dyv;
            r2    = {15'd0, cr} * {15'd0, cr};

            if (dist2 <= r2)
                tree_at = 2'b01;
            else if ((px >= tx - {5'd0, tw >> 1}) && (px <= tx + {5'd0, tw >> 1}) &&
                     (py >= ty - {5'd0, th}) && (py <= ty))
                tree_at = 2'b10;
            else
                tree_at = 2'b00;
        end
    endfunction

    // Left side, horizon -> camera (smaller/farther to bigger/closer)
    localparam T1_TX = 190, T1_TY = 145, T1_CR = 7,  T1_TW = 5, T1_TH = 8;
    localparam T2_TX = 150, T2_TY = 170, T2_CR = 9,  T2_TW = 6, T2_TH = 10;
    localparam T3_TX = 70,  T3_TY = 200, T3_CR = 12, T3_TW = 7, T3_TH = 13;
    localparam T4_TX = 105, T4_TY = 232, T4_CR = 15, T4_TW = 8, T4_TH = 16;
    localparam T5_TX = 130, T5_TY = 255, T5_CR = 18, T5_TW = 9, T5_TH = 19;

    // Right side, horizon -> camera
    localparam T6_TX  = 460, T6_TY  = 148, T6_CR  = 8,  T6_TW  = 5, T6_TH  = 9;
    localparam T7_TX  = 500, T7_TY  = 175, T7_CR  = 10, T7_TW  = 6, T7_TH  = 11;
    localparam T8_TX  = 580, T8_TY  = 205, T8_CR  = 13, T8_TW  = 7, T8_TH  = 14;
    localparam T9_TX  = 545, T9_TY  = 236, T9_CR  = 16, T9_TW  = 8, T9_TH  = 17;
    localparam T10_TX = 515, T10_TY = 258, T10_CR = 19, T10_TW = 9, T10_TH = 20;

    // Each tree's y-position is scrolled and wrapped every frame so it
    // cycles endlessly toward the camera instead of sitting still.
    wire [9:0] t1_ty, t2_ty, t3_ty, t4_ty, t5_ty, t6_ty, t7_ty, t8_ty, t9_ty, t10_ty;
    assign t1_ty  = wrap_ty(T1_TY,  scroll_y);
    assign t2_ty  = wrap_ty(T2_TY,  scroll_y);
    assign t3_ty  = wrap_ty(T3_TY,  scroll_y);
    assign t4_ty  = wrap_ty(T4_TY,  scroll_y);
    assign t5_ty  = wrap_ty(T5_TY,  scroll_y);
    assign t6_ty  = wrap_ty(T6_TY,  scroll_y);
    assign t7_ty  = wrap_ty(T7_TY,  scroll_y);
    assign t8_ty  = wrap_ty(T8_TY,  scroll_y);
    assign t9_ty  = wrap_ty(T9_TY,  scroll_y);
    assign t10_ty = wrap_ty(T10_TY, scroll_y);

    wire [1:0] tp1, tp2, tp3, tp4, tp5, tp6, tp7, tp8, tp9, tp10;
    assign tp1  = tree_at(x, y, T1_TX,  t1_ty,  T1_CR,  T1_TW,  T1_TH);
    assign tp2  = tree_at(x, y, T2_TX,  t2_ty,  T2_CR,  T2_TW,  T2_TH);
    assign tp3  = tree_at(x, y, T3_TX,  t3_ty,  T3_CR,  T3_TW,  T3_TH);
    assign tp4  = tree_at(x, y, T4_TX,  t4_ty,  T4_CR,  T4_TW,  T4_TH);
    assign tp5  = tree_at(x, y, T5_TX,  t5_ty,  T5_CR,  T5_TW,  T5_TH);
    assign tp6  = tree_at(x, y, T6_TX,  t6_ty,  T6_CR,  T6_TW,  T6_TH);
    assign tp7  = tree_at(x, y, T7_TX,  t7_ty,  T7_CR,  T7_TW,  T7_TH);
    assign tp8  = tree_at(x, y, T8_TX,  t8_ty,  T8_CR,  T8_TW,  T8_TH);
    assign tp9  = tree_at(x, y, T9_TX,  t9_ty,  T9_CR,  T9_TW,  T9_TH);
    assign tp10 = tree_at(x, y, T10_TX, t10_ty, T10_CR, T10_TW, T10_TH);

    wire tree_canopy, tree_trunk;
    assign tree_canopy = (tp1==2'b01)||(tp2==2'b01)||(tp3==2'b01)||(tp4==2'b01)||(tp5==2'b01)||
                          (tp6==2'b01)||(tp7==2'b01)||(tp8==2'b01)||(tp9==2'b01)||(tp10==2'b01);
    assign tree_trunk  = (tp1==2'b10)||(tp2==2'b10)||(tp3==2'b10)||(tp4==2'b10)||(tp5==2'b10)||
                          (tp6==2'b10)||(tp7==2'b10)||(tp8==2'b10)||(tp9==2'b10)||(tp10==2'b10);
	 // ================= Background ROM =================
	wire [16:0] bg_addr;
	wire [11:0] bg_pixel;

	// Gambar VGA 640
	assign bg_addr = (y < 120) ? (y * 640 + x) : 17'd0;

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
    wire [16:0] bg_horizon_addr;
    wire [11:0] bg_horizon_pixel;

    assign bg_horizon_addr = 17'd119 * 17'd640 + {7'd0, x};

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
    localparam signed CAR_STEP        = 3;    // kecepatan geser per frame saat ditahan
    localparam signed CAR_OFFSET_MIN  = -256; // CAR_X minimum = 256 + (-256) = 0
    localparam signed CAR_OFFSET_MAX  =  256; // CAR_X maksimum = 256 + 256 = 512 (512+128=640, pas di tepi)

    reg signed [10:0] car_offset;

    always @(posedge clk) begin
        if (x == 10'd0 && y == 10'd0) begin   // sekali per frame
            if (steer_left && !steer_right) begin
                if (car_offset > CAR_OFFSET_MIN)
                    car_offset <= car_offset - CAR_STEP;
            end else if (steer_right && !steer_left) begin
                if (car_offset < CAR_OFFSET_MAX)
                    car_offset <= car_offset + CAR_STEP;
            end
            // Tidak ada tombol ditahan (atau dua-duanya) -> diam di tempat.
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

            else if (tree_canopy) begin
                // Cooler teal-green so the foliage sits comfortably
                // next to the lavender mountains.
                terrain_r = 8'd48;
                terrain_g = 8'd98;
                terrain_b = 8'd112;
            end

            else if (tree_trunk) begin
                // Trunk warmed toward mauve rather than plain brown,
                // echoing the shoulder and mountain shadow tones.
                terrain_r = 8'd88;
                terrain_g = 8'd66;
                terrain_b = 8'd84;
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
    end

endmodule
