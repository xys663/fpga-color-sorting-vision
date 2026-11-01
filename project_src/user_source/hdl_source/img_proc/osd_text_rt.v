// =============================================================================
// osd_text_rt.v  --  通用 8x8 点阵文字叠加模块（运行期可改字符串）
// 用于把一串 ASCII 字符渲染成 OSD 像素。每个字符 8 行 8 列，逐字符排布。
// 该模块仅做"该像素是否点亮"的组合判断，颜色由上层决定。
// =============================================================================
module osd_text_rt #(
    parameter N = 4            // 字符个数
)(
    input  wire [11:0] I_x,    // 当前像素列坐标（窗口坐标）
    input  wire [11:0] I_y,    // 当前像素行坐标
    input  wire [11:0] I_x0,   // 文字左上角列
    input  wire [11:0] I_y0,   // 文字左上角行
    input  wire [N*7-1:0] I_chars, // 字符编码（每个 7bit，高位在前）
    output reg         O_on          // 该像素是否属于文字笔画
);

    // -------------------------------------------------------------------------
    // 内置 8x8 字体（仅覆盖本设计用到的字符）。row=0 为最上边一行。
    // 返回 8 位，bit7 为最左列。
    // -------------------------------------------------------------------------
    function [7:0] font_row;
        input [6:0] ch;
        input [2:0] row;
        reg [7:0] pat;
        begin
            pat = 8'h00;
            case (ch)
            // A
            7'h41: case(row)
                0:pat=8'h3C; 1:pat=8'h66; 2:pat=8'h66; 3:pat=8'h7E;
                4:pat=8'h66; 5:pat=8'h66; 6:pat=8'h66; 7:pat=8'h66; endcase
            // B
            7'h42: case(row)
                0:pat=8'h7C; 1:pat=8'h66; 2:pat=8'h66; 3:pat=8'h7C;
                4:pat=8'h66; 5:pat=8'h66; 6:pat=8'h66; 7:pat=8'h7C; endcase
            // C
            7'h43: case(row)
                0:pat=8'h3C; 1:pat=8'h66; 2:pat=8'h60; 3:pat=8'h60;
                4:pat=8'h60; 5:pat=8'h60; 6:pat=8'h66; 7:pat=8'h3C; endcase
            // D
            7'h44: case(row)
                0:pat=8'h78; 1:pat=8'h6C; 2:pat=8'h66; 3:pat=8'h66;
                4:pat=8'h66; 5:pat=8'h66; 6:pat=8'h6C; 7:pat=8'h78; endcase
            // E
            7'h45: case(row)
                0:pat=8'h7E; 1:pat=8'h60; 2:pat=8'h60; 3:pat=8'h7C;
                4:pat=8'h60; 5:pat=8'h60; 6:pat=8'h60; 7:pat=8'h7E; endcase
            // F
            7'h46: case(row)
                0:pat=8'h7E; 1:pat=8'h60; 2:pat=8'h60; 3:pat=8'h7C;
                4:pat=8'h60; 5:pat=8'h60; 6:pat=8'h60; 7:pat=8'h60; endcase
            // G
            7'h47: case(row)
                0:pat=8'h3C; 1:pat=8'h66; 2:pat=8'h60; 3:pat=8'h6E;
                4:pat=8'h66; 5:pat=8'h66; 6:pat=8'h66; 7:pat=8'h3C; endcase
            // H
            7'h48: case(row)
                0:pat=8'h66; 1:pat=8'h66; 2:pat=8'h66; 3:pat=8'h7E;
                4:pat=8'h66; 5:pat=8'h66; 6:pat=8'h66; 7:pat=8'h66; endcase
            // I
            7'h49: case(row)
                0:pat=8'h7E; 1:pat=8'h18; 2:pat=8'h18; 3:pat=8'h18;
                4:pat=8'h18; 5:pat=8'h18; 6:pat=8'h18; 7:pat=8'h7E; endcase
            // K
            7'h4B: case(row)
                0:pat=8'h66; 1:pat=8'h66; 2:pat=8'h6C; 3:pat=8'h78;
                4:pat=8'h78; 5:pat=8'h6C; 6:pat=8'h66; 7:pat=8'h66; endcase
            // L
            7'h4C: case(row)
                0:pat=8'h60; 1:pat=8'h60; 2:pat=8'h60; 3:pat=8'h60;
                4:pat=8'h60; 5:pat=8'h60; 6:pat=8'h60; 7:pat=8'h7E; endcase
            // M
            7'h4D: case(row)
                0:pat=8'h63; 1:pat=8'h77; 2:pat=8'h7F; 3:pat=8'h6B;
                4:pat=8'h63; 5:pat=8'h63; 6:pat=8'h63; 7:pat=8'h63; endcase
            // N
            7'h4E: case(row)
                0:pat=8'h66; 1:pat=8'h76; 2:pat=8'h7E; 3:pat=8'h6E;
                4:pat=8'h66; 5:pat=8'h66; 6:pat=8'h66; 7:pat=8'h66; endcase
            // O
            7'h4F: case(row)
                0:pat=8'h3C; 1:pat=8'h66; 2:pat=8'h66; 3:pat=8'h66;
                4:pat=8'h66; 5:pat=8'h66; 6:pat=8'h66; 7:pat=8'h3C; endcase
            // P
            7'h50: case(row)
                0:pat=8'h7C; 1:pat=8'h66; 2:pat=8'h66; 3:pat=8'h7C;
                4:pat=8'h60; 5:pat=8'h60; 6:pat=8'h60; 7:pat=8'h60; endcase
            // R
            7'h52: case(row)
                0:pat=8'h7C; 1:pat=8'h66; 2:pat=8'h66; 3:pat=8'h7C;
                4:pat=8'h68; 5:pat=8'h64; 6:pat=8'h62; 7:pat=8'h66; endcase
            // S
            7'h53: case(row)
                0:pat=8'h3C; 1:pat=8'h66; 2:pat=8'h60; 3:pat=8'h3C;
                4:pat=8'h06; 5:pat=8'h06; 6:pat=8'h66; 7:pat=8'h3C; endcase
            // T
            7'h54: case(row)
                0:pat=8'h7E; 1:pat=8'h18; 2:pat=8'h18; 3:pat=8'h18;
                4:pat=8'h18; 5:pat=8'h18; 6:pat=8'h18; 7:pat=8'h18; endcase
            // U
            7'h55: case(row)
                0:pat=8'h66; 1:pat=8'h66; 2:pat=8'h66; 3:pat=8'h66;
                4:pat=8'h66; 5:pat=8'h66; 6:pat=8'h66; 7:pat=8'h3C; endcase
            // W
            7'h57: case(row)
                0:pat=8'h63; 1:pat=8'h63; 2:pat=8'h63; 3:pat=8'h6B;
                4:pat=8'h7F; 5:pat=8'h77; 6:pat=8'h66; 7:pat=8'h66; endcase
            // Y
            7'h59: case(row)
                0:pat=8'h66; 1:pat=8'h66; 2:pat=8'h3C; 3:pat=8'h18;
                4:pat=8'h18; 5:pat=8'h18; 6:pat=8'h18; 7:pat=8'h18; endcase
            // X（2026-10-28 新增：用于 "XY" 坐标行）
            7'h58: case(row)
                0:pat=8'h66; 1:pat=8'h66; 2:pat=8'h3C; 3:pat=8'h18;
                4:pat=8'h18; 5:pat=8'h3C; 6:pat=8'h66; 7:pat=8'h66; endcase
            // 0-9
            7'h30: case(row)
                0:pat=8'h3C; 1:pat=8'h66; 2:pat=8'h6E; 3:pat=8'h76;
                4:pat=8'h66; 5:pat=8'h66; 6:pat=8'h66; 7:pat=8'h3C; endcase
            7'h31: case(row)
                0:pat=8'h18; 1:pat=8'h38; 2:pat=8'h18; 3:pat=8'h18;
                4:pat=8'h18; 5:pat=8'h18; 6:pat=8'h18; 7:pat=8'h3C; endcase
            7'h32: case(row)
                0:pat=8'h3C; 1:pat=8'h66; 2:pat=8'h66; 3:pat=8'h1C;
                4:pat=8'h30; 5:pat=8'h60; 6:pat=8'h60; 7:pat=8'h7E; endcase
            7'h33: case(row)
                0:pat=8'h7E; 1:pat=8'h30; 2:pat=8'h18; 3:pat=8'h0C;
                4:pat=8'h06; 5:pat=8'h66; 6:pat=8'h66; 7:pat=8'h3C; endcase
            7'h34: case(row)
                0:pat=8'h0C; 1:pat=8'h1C; 2:pat=8'h3C; 3:pat=8'h6C;
                4:pat=8'h7E; 5:pat=8'h0C; 6:pat=8'h0C; 7:pat=8'h0C; endcase
            7'h35: case(row)
                0:pat=8'h7E; 1:pat=8'h60; 2:pat=8'h7C; 3:pat=8'h06;
                4:pat=8'h06; 5:pat=8'h66; 6:pat=8'h66; 7:pat=8'h3C; endcase
            7'h36: case(row)
                0:pat=8'h3C; 1:pat=8'h60; 2:pat=8'h60; 3:pat=8'h7C;
                4:pat=8'h66; 5:pat=8'h66; 6:pat=8'h66; 7:pat=8'h3C; endcase
            7'h37: case(row)
                0:pat=8'h7E; 1:pat=8'h06; 2:pat=8'h0C; 3:pat=8'h18;
                4:pat=8'h30; 5:pat=8'h30; 6:pat=8'h30; 7:pat=8'h30; endcase
            7'h38: case(row)
                0:pat=8'h3C; 1:pat=8'h66; 2:pat=8'h66; 3:pat=8'h3C;
                4:pat=8'h66; 5:pat=8'h66; 6:pat=8'h66; 7:pat=8'h3C; endcase
            7'h39: case(row)
                0:pat=8'h3C; 1:pat=8'h66; 2:pat=8'h66; 3:pat=8'h66;
                4:pat=8'h3E; 5:pat=8'h06; 6:pat=8'h06; 7:pat=8'h3C; endcase
            // ':'
            7'h3A: case(row)
                0:pat=8'h00; 1:pat=8'h00; 2:pat=8'h18; 3:pat=8'h18;
                4:pat=8'h00; 5:pat=8'h00; 6:pat=8'h18; 7:pat=8'h18; endcase
            // ','（2026-10-28 新增：坐标 X,Y 分隔）
            7'h2C: case(row)
                0:pat=8'h00; 1:pat=8'h00; 2:pat=8'h00; 3:pat=8'h00;
                4:pat=8'h00; 5:pat=8'h18; 6:pat=8'h18; 7:pat=8'h30; endcase
            // '.'（2026-10-28 新增：延迟小数 16.6）
            7'h2E: case(row)
                0:pat=8'h00; 1:pat=8'h00; 2:pat=8'h00; 3:pat=8'h00;
                4:pat=8'h00; 5:pat=8'h00; 6:pat=8'h18; 7:pat=8'h18; endcase
            // '='
            7'h3D: case(row)
                0:pat=8'h00; 1:pat=8'h00; 2:pat=8'h7E; 3:pat=8'h00;
                4:pat=8'h00; 5:pat=8'h7E; 6:pat=8'h00; 7:pat=8'h00; endcase
            // ' '
            7'h20: case(row)
                0:pat=8'h00; 1:pat=8'h00; 2:pat=8'h00; 3:pat=8'h00;
                4:pat=8'h00; 5:pat=8'h00; 6:pat=8'h00; 7:pat=8'h00; endcase
            // 其它默认显示为 '?'
            default: case(row)
                0:pat=8'h18; 1:pat=8'h24; 2:pat=8'h24; 3:pat=8'h0C;
                4:pat=8'h18; 5:pat=8'h24; 6:pat=8'h24; 7:pat=8'h18; endcase
            endcase
            font_row = pat;
        end
    endfunction

    integer idx;
    integer i;
    reg [6:0] ch;
    reg [2:0] row;
    reg [2:0] col;
    reg [7:0] bits;

    // -------------------------------------------------------------------------
    // 2026-10-30 修正：DSP 超编（46/40，布局报 PHY-8023 Can not place Macro）
    //
    //   原来取字符用的是变量移位：
    //       ch = (I_chars >> ((N-1-idx)*7)) & 7'h7F;
    //   其中 (N-1-idx)*7 是"变量乘以常数 7"、外面再套一个"变量桶形移位"，
    //   TD 6.2 会把这两样各综合成一个 DSP 乘法宏 —— 每个文字实例吃 2 个 DSP。
    //   本设计有 14 个文字实例（MODE/FPS/F/OBJ/N/XY/DLY 等），合计 28 个 DSP，
    //   加上 hdmi_mixer 自身的 16 个、color_obj_detect 的 2 个共 46 个，
    //   超过 HX1P35A 的 40 个，导致 Macro 合法化失败、无法布局。
    //
    //   改法：把 N 个候选字符写成一个 for 循环的"逐个比较 + 常数移位"，
    //   循环边界是常数 N，综合器展开后每次移位的量都是常数，
    //   于是整段变成纯 LUT 的 N 选 1 选择器，**0 个 DSP**。
    //   代价：每个实例约 20 个 LUT6，14 个约 300 个，LUT 占用仍有余量。
    // -------------------------------------------------------------------------
    always @(*) begin
        O_on = 1'b0;
        idx  = (I_x >= I_x0) ? ((I_x - I_x0) >> 3) : 32'd0;
        row  = I_y - I_y0;
        col  = (I_x - I_x0) - (idx[2:0] << 3);
        ch   = 7'h20;                       // 默认空格

        // 移位量 7*(N-1-i) 在循环展开后为常数 => 不会生成移位器/DSP
        for (i = 0; i < N; i = i + 1)
            if (idx == i)
                ch = (I_chars >> (7*(N-1-i))) & 7'h7F;

        if (I_y >= I_y0 && I_y < (I_y0 + 12'd8) &&
            I_x >= I_x0 && idx < N)
            begin
                bits = font_row(ch, row);
                O_on = bits[7-col];
            end
    end

endmodule
