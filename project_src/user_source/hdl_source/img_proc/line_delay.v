// =============================================================================
// line_delay.v  --  整行延迟（像素数 = DEPTH），用同步读 BRAM 推断实现
// 在每个 I_pix_en 上升沿：读出当前地址"旧值"（即 DEPTH 个像素之前写入的值），
// 同时写入当前像素并推进地址。从而稳定产生恰好一整行的延迟。
// 同步读写法可被 TD 正确推断为 Block RAM，避免组合读被误判为锁存器/巨型多路选择器。
// =============================================================================
module line_delay #(
    parameter WIDTH = 8,
    parameter DEPTH = 1280,        // 一行像素数
    parameter AW    = 11           // 地址位宽 = clog2(DEPTH)
)(
    input  wire             I_clk,
    input  wire             I_pix_en,
    input  wire [WIDTH-1:0] I_din,
    output reg  [WIDTH-1:0] O_dout
);
    reg [AW-1:0]           addr;
    reg [WIDTH-1:0]        mem [0:DEPTH-1];

    always @(posedge I_clk) begin
        if (I_pix_en) begin
            O_dout <= mem[addr];   // 读出上一次写入该地址的值（DEPTH 拍之前）
            mem[addr] <= I_din;    // 写入当前像素
            // 关键修复：地址必须在 DEPTH 处回卷，而不是地址位宽 AW 的满量程(2^AW)。
            // 本设计一行恰为 DEPTH(=1280) 个有效像素；只有在此精确回卷，延迟才等于
            // "恰好一整行"。若按 AW=11 默认在 2048 回卷，"上一行"实际错位 768 像素，
            // Sobel 上下行不相关 -> 全屏盐噪声 + 竖向接缝（把画面撕成三块）。
            addr <= (addr == (DEPTH - 1)) ? {AW{1'b0}} : addr + 1'b1;
        end
    end

endmodule
