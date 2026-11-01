// =============================================================================
// img_algo.v  --  图像处理算法（组合逻辑）
// 输入：当前像素 RGB、以当前像素为中心的 3x3 灰度窗口、模式选择
// 输出：处理后的 24bit RGB
//   mode 0 RAW   : 原图
//   mode 1 GRAY  : 灰度
//   mode 2 BIN   : 固定阈值二值化
//   mode 3 EDGE  : Sobel 边缘（黑白）
//   mode 4 TOON  : 卡通化（边缘变黑 + 颜色量化）
//   mode 5 FIND  : 原图（目标框由 OSD 叠加）
// =============================================================================
module img_algo (
    input  wire [2:0]  I_mode,
    input  wire [23:0] I_rgb,
    input  wire [7:0]  p00, p01, p02,
    input  wire [7:0]  p10, p11, p12,
    input  wire [7:0]  p20, p21, p22,
    output wire [23:0] O_rgb
);

    function [7:0] rgb2gray;
        input [23:0] c;
        reg [15:0] s;
        begin
            // 用 16bit 常量做乘法，避免 8bit 上下文截断
            s = c[23:16]*16'd77 + c[15:8]*16'd150 + c[7:0]*16'd29;
            rgb2gray = s[15:8];
        end
    endfunction

    wire [7:0] gc = rgb2gray(I_rgb);

    // ---- Sobel ----
    reg signed [11:0] gx, gy;
    reg signed [11:0] agx, agy;
    wire [11:0] mag;
    wire signed [11:0] s00,s01,s02,s10,s11,s12,s20,s21,s22;
    assign s00 = $signed({4'b0,p00}); assign s01 = $signed({4'b0,p01}); assign s02 = $signed({4'b0,p02});
    assign s10 = $signed({4'b0,p10}); assign s11 = $signed({4'b0,p11}); assign s12 = $signed({4'b0,p12});
    assign s20 = $signed({4'b0,p20}); assign s21 = $signed({4'b0,p21}); assign s22 = $signed({4'b0,p22});
    always @(*) begin
        gx = (s20 + (s21<<1) + s22) - (s00 + (s01<<1) + s02);
        gy = (s00 + (s10<<1) + s20) - (s02 + (s12<<1) + s22);
        agx = gx[11] ? -gx : gx;
        agy = gy[11] ? -gy : gy;
    end
    assign mag = agx + agy;

    // ---- 卡通量化 ----
    wire [7:0] rq = {I_rgb[23:22], I_rgb[23:22], I_rgb[23:22], I_rgb[23:22]};
    wire [7:0] gq = {I_rgb[15:14], I_rgb[15:14], I_rgb[15:14], I_rgb[15:14]};
    wire [7:0] bq = {I_rgb[7:6],   I_rgb[7:6],   I_rgb[7:6],   I_rgb[7:6]};

    reg [23:0] res;
    always @(*) begin
        case (I_mode)
            3'd0: res = I_rgb;
            3'd1: res = {gc, gc, gc};
            3'd2: res = (gc >= 8'd128) ? 24'hFFFFFF : 24'h000000;
            3'd3: res = (mag >= 12'd80) ? 24'hFFFFFF : 24'h000000;
            3'd4: res = (mag >= 12'd80) ? 24'h000000 : {rq, gq, bq};
            3'd5: res = I_rgb;
            default: res = I_rgb;
        endcase
    end
    assign O_rgb = res;

endmodule
