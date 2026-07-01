module render_static (
    input  wire       video_on,
    input  wire [9:0] x,
    input  wire [9:0] y,
    output reg  [7:0] red,
    output reg  [7:0] green,
    output reg  [7:0] blue
);

    wire [9:0] dy;
    wire [9:0] road_left;
    wire [9:0] road_right;
    wire [9:0] road_width;
    wire [9:0] line_margin;

    assign dy = (y >= 10'd120) ? (y - 10'd120) : 10'd0;

    assign road_left  = 10'd260 - (dy >> 1);
    assign road_right = 10'd380 + (dy >> 1);

    // Margin scales the road's
	 
    assign road_width  = road_right - road_left;
    assign line_margin = road_width >> 6;   // ~1/16th of current width

    wire road_area;
    wire left_solid_line;
    wire right_solid_line;
    wire center_dash;
    wire car_body;
    wire car_window;
    wire car_tire_left;
    wire car_tire_right;

    // Dirt/gravel shoulder: a narrow strip just outside the asphalt on
    // both sides, so the road doesn't cut straight to grass.
    localparam SHOULDER_WIDTH = 6;
    wire shoulder_left, shoulder_right;

    // A touch of texture on the asphalt itself so it doesn't look like
    // a single flat painted block. Cheap and synthesizable: just XOR a
    // handful of position bits together to get a fixed speckle pattern.
    wire asphalt_fleck;

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

    wire [1:0] tp1, tp2, tp3, tp4, tp5, tp6, tp7, tp8, tp9, tp10;
    assign tp1  = tree_at(x, y, T1_TX,  T1_TY,  T1_CR,  T1_TW,  T1_TH);
    assign tp2  = tree_at(x, y, T2_TX,  T2_TY,  T2_CR,  T2_TW,  T2_TH);
    assign tp3  = tree_at(x, y, T3_TX,  T3_TY,  T3_CR,  T3_TW,  T3_TH);
    assign tp4  = tree_at(x, y, T4_TX,  T4_TY,  T4_CR,  T4_TW,  T4_TH);
    assign tp5  = tree_at(x, y, T5_TX,  T5_TY,  T5_CR,  T5_TW,  T5_TH);
    assign tp6  = tree_at(x, y, T6_TX,  T6_TY,  T6_CR,  T6_TW,  T6_TH);
    assign tp7  = tree_at(x, y, T7_TX,  T7_TY,  T7_CR,  T7_TW,  T7_TH);
    assign tp8  = tree_at(x, y, T8_TX,  T8_TY,  T8_CR,  T8_TW,  T8_TH);
    assign tp9  = tree_at(x, y, T9_TX,  T9_TY,  T9_CR,  T9_TW,  T9_TH);
    assign tp10 = tree_at(x, y, T10_TX, T10_TY, T10_CR, T10_TW, T10_TH);

    wire tree_canopy, tree_trunk;
    assign tree_canopy = (tp1==2'b01)||(tp2==2'b01)||(tp3==2'b01)||(tp4==2'b01)||(tp5==2'b01)||
                          (tp6==2'b01)||(tp7==2'b01)||(tp8==2'b01)||(tp9==2'b01)||(tp10==2'b01);
    assign tree_trunk  = (tp1==2'b10)||(tp2==2'b10)||(tp3==2'b10)||(tp4==2'b10)||(tp5==2'b10)||
                          (tp6==2'b10)||(tp7==2'b10)||(tp8==2'b10)||(tp9==2'b10)||(tp10==2'b10);
    assign asphalt_fleck = x[1] ^ x[4] ^ y[2] ^ y[5];

 

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
                             (x <= road_left + line_margin + 10'd8);

    assign right_solid_line = road_area &&
                              (x >= road_right - line_margin - 10'd8) &&
                              (x <= road_right - line_margin);

    assign center_dash = road_area &&
                         (x >= 10'd317) &&
                         (x <= 10'd323) &&
                         (y[5] == 1'b0);

    assign car_body = (x >= 10'd285) && (x <= 10'd355) &&
                      (y >= 10'd370) && (y <= 10'd450);

    assign car_window = (x >= 10'd305) && (x <= 10'd335) &&
                        (y >= 10'd382) && (y <= 10'd405);

    assign car_tire_left = (x >= 10'd275) && (x <= 10'd292) &&
                           (y >= 10'd415) && (y <= 10'd455);

    assign car_tire_right = (x >= 10'd348) && (x <= 10'd365) &&
                            (y >= 10'd415) && (y <= 10'd455);

   
    assign shoulder_left  = (y >= 10'd120) &&
                             (x >= road_left - SHOULDER_WIDTH) &&
                             (x <  road_left);

    assign shoulder_right = (y >= 10'd120) &&
                             (x >  road_right) &&
                             (x <= road_right + SHOULDER_WIDTH);

    always @(*) begin
        if (!video_on) begin
            red   = 8'd0;
            green = 8'd0;
            blue  = 8'd0;
        end

        else if (car_tire_left || car_tire_right) begin
            red   = 8'd5;
            green = 8'd5;
            blue  = 8'd5;
        end

        else if (car_window) begin
            red   = 8'd120;
            green = 8'd190;
            blue  = 8'd255;
        end

        else if (car_body) begin
            red   = 8'd210;
            green = 8'd20;
            blue  = 8'd20;
        end

        else if (left_solid_line || right_solid_line || center_dash) begin
            red   = 8'd255;
            green = 8'd255;
            blue  = 8'd255;
        end

        else if (road_area) begin
            if (asphalt_fleck) begin
                red   = 8'd70;
                green = 8'd70;
                blue  = 8'd70;
            end else begin
                red   = 8'd60;
                green = 8'd60;
                blue  = 8'd60;
            end
        end

        else if (shoulder_left || shoulder_right) begin
            red   = 8'd120;
            green = 8'd100;
            blue  = 8'd70;
        end

        else if (y < 10'd120) begin
            if (mtn_back1) begin
                red   = 8'd130;
                green = 8'd140;
                blue  = 8'd170;
            end

            else begin
                red   = 8'd100;
                green = 8'd190;
                blue  = 8'd255;
            end
        end

        else if (tree_canopy) begin
            red   = 8'd30;
            green = 8'd95;
            blue  = 8'd35;
        end

        else if (tree_trunk) begin
            red   = 8'd90;
            green = 8'd55;
            blue  = 8'd25;
        end

        else begin
            red   = 8'd45;
            green = 8'd150;
            blue  = 8'd60;
        end
    end

endmodule