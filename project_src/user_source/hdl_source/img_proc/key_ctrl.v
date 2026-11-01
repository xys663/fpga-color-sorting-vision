// =============================================================================
// key_ctrl.v  --  4 按键消抖 + 模式/OSD/目标颜色控制 + 白平衡标定
// 按键为低电平有效（板上拉）。每个按键消抖后产生一个时钟宽度的"按下脉冲"。
//   KEY1(最低位): 模式 +1（0..5 循环）
//   KEY2       : 模式 -1
//   KEY3       : OSD 叠加 开关
//   KEY4       : 目标检测颜色 切换
// 组合键（消抖后两键同时按住，上升沿触发一次）：
//   KEY1+KEY2  : 触发白平衡一键标定（镜头先对准白纸）
//   KEY3+KEY4  : 复位白平衡增益为 1.0（旁路）
// 具体物理引脚见 pin.adc 与《按键说明.md》。
// =============================================================================
module key_ctrl (
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire [3:0] I_button,     // bit0=KEY1 ... bit3=KEY4，低有效
    output reg  [2:0] O_mode,
    output reg        O_osd_en,
    output reg  [2:0] O_target_sel,
    output reg        O_cal,        // KEY1+KEY2：白平衡标定触发（1 拍脉冲）
    output reg        O_wb_rst      // KEY3+KEY4：白平衡增益复位（1 拍脉冲）
);

    reg  [3:0] sync1, sync2;
    reg  [23:0] cnt [0:3];
    reg  [3:0] low, low_d;
    wire [3:0] pulse;
    integer i;

    // 组合键检测：消抖后两键同时按住（上升沿产生一次脉冲）
    wire       both01   = low[0] & low[1];
    wire       both23   = low[2] & low[3];
    reg        both01_d, both23_d;
    wire       both01_p = both01 & ~both01_d;   // KEY1+KEY2 上升沿
    wire       both23_p = both23 & ~both23_d;   // KEY3+KEY4 上升沿

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            sync1 <= 4'b1111; sync2 <= 4'b1111;
            for (i = 0; i < 4; i = i + 1) cnt[i] <= 24'd0;
            low <= 4'd0; low_d <= 4'd0;
            both01_d <= 1'b0; both23_d <= 1'b0;
        end else begin
            sync1 <= I_button; sync2 <= sync1;
            for (i = 0; i < 4; i = i + 1) begin
                if (sync2[i] == 1'b0)
                    cnt[i] <= (cnt[i] < 24'hFFFFF) ? cnt[i] + 1'b1 : cnt[i];
                else
                    cnt[i] <= 24'd0;
                low[i] <= (cnt[i] >= 24'd200000);   // ≈2.7ms @74MHz
            end
            low_d <= low;
            both01_d <= both01;
            both23_d <= both23;
        end
    end

    assign pulse = low & ~low_d;   // 每个按键的"按下脉冲"

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_mode <= 3'd0; O_osd_en <= 1'b1; O_target_sel <= 3'd0;
            O_cal <= 1'b0; O_wb_rst <= 1'b0;
        end else begin
            // 默认不输出脉冲，避免 latch
            O_cal    <= 1'b0;
            O_wb_rst <= 1'b0;

            // ---- KEY1+KEY2：白平衡标定（按住期间屏蔽模式 +/-）----
            if (both01) begin
                O_cal <= both01_p;
            end else begin
                // 模式 0..6：0 RAW 1 GRAY 2 BIN 3 EDGE 4 TOON 5 FIND 6 MOTION
                //   （模式6 原为 DEFECT 缺陷检测，2026-10-30 换成帧差运动检测）
                if (pulse[0]) O_mode <= (O_mode == 3'd6) ? 3'd0 : O_mode + 1'b1; // 模式+
                if (pulse[1]) O_mode <= (O_mode == 3'd0) ? 3'd6 : O_mode - 1'b1; // 模式-
            end

            // ---- KEY3+KEY4：白平衡复位（按住期间屏蔽 OSD/颜色切换）----
            if (both23) begin
                O_wb_rst <= both23_p;
            end else begin
                if (pulse[2]) O_osd_en    <= ~O_osd_en;                          // OSD 开关
                // 颜色切换：0 RED 1 GRN 2 BLU 3 YEL 5 WHT（跳过 4 CYN，按需求去掉青色）
                if (pulse[3]) begin
                    case (O_target_sel)
                        3'd0:    O_target_sel <= 3'd1;
                        3'd1:    O_target_sel <= 3'd2;
                        3'd2:    O_target_sel <= 3'd3;
                        3'd3:    O_target_sel <= 3'd5;   // 跳过 4(CYN)
                        3'd4:    O_target_sel <= 3'd5;   // 保险：正常流程不会到 4
                        default: O_target_sel <= 3'd0;   // 5 -> 0
                    endcase
                end
            end
        end
    end

endmodule
