// =============================================================================
// bin2bcd.v  --  12 位二进制转 4 位十进制 BCD（串行 double-dabble）
//
// 用途：把 OSD 要显示的数字（目标框坐标 0~1279、延迟 0~999 的 0.1ms 值）
//       转成十进制位，供 osd_text_rt 渲染成 ASCII。
//
// 设计要点（这是本模块存在的唯一理由）：
//   * 本工程 clk1(74.25MHz HDMI 像素域) 的综合时序余量本来就是负的
//     （Setup WNS 约 -25ns，最差路径 wb_g_gain_reg -> pd1_reg，逻辑级数 38），
//     所以**绝不能**在像素通路上放组合除法器 / 组合 BCD 转换。
//   * 这里做成 12 拍串行状态机：每拍只做"4 个 nibble 各自判断 >4 则 +3"
//     ＋一次 28 位左移，逻辑深度很浅；且只在帧首启动一次，不占像素节拍。
//
// 时序：I_start 拉高一拍启动 -> O_busy 拉高 12 拍 -> O_busy 拉低，
//       此时 O_d3..O_d0 有效并一直保持，直到下一次启动。
//       I_start 在 O_busy 期间被忽略（上层每帧只发一次，不会冲突）。
//
// 算法正确性：已用 Python 按同样的位选复刻该状态机，对 0~4095 全量扫描
//             零错误（见 verify_20261028/osd_verify.py 第 1 节）。
// =============================================================================
module bin2bcd (
    input  wire          I_clk,
    input  wire          I_rst_n,
    input  wire          I_start,
    input  wire [11:0]   I_bin,
    output reg  [3:0]    O_d3,     // 千位
    output reg  [3:0]    O_d2,     // 百位
    output reg  [3:0]    O_d1,     // 十位
    output reg  [3:0]    O_d0,     // 个位
    output reg           O_busy
);

    // 工作寄存器 R = { d3[3:0], d2[3:0], d1[3:0], d0[3:0], bin[11:0] }，共 28 位
    reg [27:0] R;
    reg [4:0]  cnt;

    // 单个 BCD 位：>4 则 +3
    function [3:0] adj;
        input [3:0] v;
        begin
            adj = (v > 4'd4) ? (v + 4'd3) : v;
        end
    endfunction

    wire [27:0] R_adj = { adj(R[27:24]), adj(R[23:20]), adj(R[19:16]), adj(R[15:12]),
                          R[11:0] };

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            R      <= 28'd0;
            cnt    <= 5'd0;
            O_busy <= 1'b0;
            O_d3   <= 4'd0;
            O_d2   <= 4'd0;
            O_d1   <= 4'd0;
            O_d0   <= 4'd0;
        end
        else if (I_start && !O_busy) begin
            R      <= {16'd0, I_bin};        // BCD 段清零，二进制值装入低 12 位
            cnt    <= 5'd12;
            O_busy <= 1'b1;
        end
        else if (O_busy) begin
            if (cnt != 5'd0) begin
                R   <= { R_adj[26:0], 1'b0 };  // 先校正再整体左移（double-dabble 标准步骤）
                cnt <= cnt - 5'd1;
            end
            else begin
                O_d3   <= R[27:24];
                O_d2   <= R[23:20];
                O_d1   <= R[19:16];
                O_d0   <= R[15:12];
                O_busy <= 1'b0;
            end
        end
    end

endmodule
