// =============================================================================
// window3x3_gray.v  --  由像素流构建以当前像素为中心的 3x3 灰度窗口
// 使用两个级联 line_delay 得到"上一行"和"上上行"，再用 2 级移位得到水平邻域。
// 输出窗口与当前像素在同一拍有效（中心 = 当前像素），便于后续处理对齐。
// =============================================================================
module window3x3_gray (
    input  wire       I_clk,
    input  wire       I_pix_en,
    input  wire [7:0] I_gray,
    output wire [7:0] p00, p01, p02,
    output wire [7:0] p10, p11, p12,
    output wire [7:0] p20, p21, p22
);

    // 当前行的水平移位：sh1 = x-1, sh2 = x-2
    reg [7:0] sh1, sh2;
    // 上一行（line n-1）
    wire [7:0] g_n1;
    reg  [7:0] g_n1_sh1, g_n1_sh2;
    // 上上行（line n-2）
    wire [7:0] g_n2;
    reg  [7:0] g_n2_sh1, g_n2_sh2;

    always @(posedge I_clk) begin
        if (I_pix_en) begin
            sh1 <= I_gray;   sh2 <= sh1;
            g_n1_sh1 <= g_n1;  g_n1_sh2 <= g_n1_sh1;
            g_n2_sh1 <= g_n2;  g_n2_sh2 <= g_n2_sh1;
        end
    end

    line_delay #(.WIDTH(8), .DEPTH(1280), .AW(11)) u_ld1 (
        .I_clk(I_clk), .I_pix_en(I_pix_en), .I_din(I_gray), .O_dout(g_n1));
    line_delay #(.WIDTH(8), .DEPTH(1280), .AW(11)) u_ld2 (
        .I_clk(I_clk), .I_pix_en(I_pix_en), .I_din(g_n1), .O_dout(g_n2));

    assign p00 = g_n2_sh2; assign p01 = g_n2_sh1; assign p02 = g_n2;
    assign p10 = g_n1_sh2; assign p11 = g_n1_sh1; assign p12 = g_n1;
    assign p20 = sh2;      assign p21 = sh1;      assign p22 = I_gray;

endmodule
