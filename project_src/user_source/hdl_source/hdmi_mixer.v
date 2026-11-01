// =============================================================================
// hdmi_mixer.v  --  增强版（含图像处理 + OSD 叠加）
// 在原例程 hdmi_mixer 基础上：
//   * 接入 3x3 Sobel/灰度/二值/卡通 等算法（img_algo）
//   * 接入 颜色目标检测（color_obj_detect）
//   * 接入 OSD 叠加（安路 Logo 横幅 + 状态文字 + 目标框 + 帧计数）
// 保留原 DEBUG_MODE 行为；DEBUG_MODE==0 时走"图像处理 + OSD"通路。
// 与原模块端口兼容，仅新增 I_mode / I_osd_en / I_target_sel 三个输入。
// =============================================================================
module hdmi_mixer #(
    parameter H_OFFSET = 0,
    parameter V_OFFSET = 0,
    parameter IMG_WIDTH = 1600,
    parameter IMG_HEIGHT = 900,
    parameter integer DEBUG_MODE = 0
)(
    input wire        I_clk,
    input wire        I_rst_n,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,

    input wire [3:0]  I_debug_status,

    output wire       O_video_rd_en,
    input wire [23:0] I_video_rd_data,

    // ---- 新增：来自按键控制的运行时参数 ----
    input wire [2:0]  I_mode,        // 0 RAW 1 GRAY 2 BIN 3 EDGE 4 TOON 5 FIND
    input wire        I_osd_en,      // OSD 总开关
    input wire [2:0]  I_target_sel,  // 目标检测颜色选择
    input wire        I_wb_cal,      // 白平衡标定触发（KEY1+KEY2 同时按下，1 拍脉冲）
    input wire        I_wb_rst,      // 白平衡增益复位（KEY3+KEY4 同时按下，1 拍脉冲）
    input wire        I_cam_fs,      // 2026-10-28 新增：采集帧起始（已在采集时钟域展宽，跨域安全）

    // ---- 缺陷检测状态（送顶层报警：蜂鸣器 + 串口上传）----
    output wire        O_mv_present,   // 上一帧是否检测到运动（有运动块）
    output wire [15:0] O_mv_cnt,       // 上一帧运动块数（160x90 网格，最大 14400）

    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg [23:0] O_hdmi_data
);

    reg        S_video_vsync_1d, S_video_vsync_2d;
    reg        S_video_hsync_1d, S_video_hsync_2d;
    reg        S_video_de_1d,    S_video_de_2d;
    reg [23:0] S_video_data_1d,  S_video_data_2d;
    reg        S_window_active_1d, S_window_active_2d;
    reg [11:0] S_x, S_y;

    wire S_window_active;
    assign S_window_active = I_video_de &&
                             (S_x >= H_OFFSET) &&
                             (S_x <  (H_OFFSET + IMG_WIDTH)) &&
                             (S_y >= V_OFFSET) &&
                             (S_y <  (V_OFFSET + IMG_HEIGHT));

    assign O_video_rd_en = S_window_active;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0; S_y <= 12'd0;
        end else begin
            if(I_video_user)
                S_y <= 12'd0;
            else if(I_video_de && I_video_last)
                S_y <= S_y + 12'd1;
            if(!I_video_de || I_video_last)
                S_x <= 12'd0;
            else
                S_x <= S_x + 12'd1;
        end
    end

    // -------------------------------------------------------------------------
    // 图像处理流水线（与原始数据通路保持相同的 2 拍延迟以对齐窗口坐标）
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // 画质增强（原地实现，避免新增源文件被 TD 漏登记成 black box）：
    //   ① gamma 0.7 折线抬暗部  ② 饱和度 x1.4375（移位加法，不占 DSP）
    //   ③ 数字增益（当前 1.0，曝光已合适，不再额外提亮）
    // 纯组合，0 额外延迟，不破坏 OSD / 同步对齐
    // -------------------------------------------------------------------------
    function [7:0] enh_gamma07;
        input [7:0] v;
        reg [7:0]  base;
        reg [7:0]  delta;
        reg [11:0] interp;
        begin
            case (v[7:4])
                4'h0: begin base = 8'd0;   delta = 8'd37; end
                4'h1: begin base = 8'd37;  delta = 8'd23; end
                4'h2: begin base = 8'd60;  delta = 8'd19; end
                4'h3: begin base = 8'd79;  delta = 8'd18; end
                4'h4: begin base = 8'd97;  delta = 8'd16; end
                4'h5: begin base = 8'd113; delta = 8'd16; end
                4'h6: begin base = 8'd129; delta = 8'd14; end
                4'h7: begin base = 8'd143; delta = 8'd14; end
                4'h8: begin base = 8'd157; delta = 8'd14; end
                4'h9: begin base = 8'd171; delta = 8'd13; end
                4'ha: begin base = 8'd184; delta = 8'd13; end
                4'hb: begin base = 8'd197; delta = 8'd12; end
                4'hc: begin base = 8'd209; delta = 8'd12; end
                4'hd: begin base = 8'd221; delta = 8'd12; end
                4'he: begin base = 8'd233; delta = 8'd11; end
                default: begin base = 8'd244; delta = 8'd11; end
            endcase
            interp = delta * v[3:0] + 8;
            if (v == 8'hFF)
                enh_gamma07 = 8'hFF;
            else
                enh_gamma07 = base + (interp >> 4);
        end
    endfunction

    // 数字增益（Q8）：由下方「自动曝光」环路逐帧调节，不再写死。
    //   复位值 1.5 接近室内常见光照，上电后 AEC 会自己收拢到最优。
    reg  [15:0] dig_gain;

    function [7:0] enh_sat;
        input [7:0]  c;
        input [7:0]  yy;
        input [15:0] dg;      // 数字增益（Q8，由 AEC 逐帧给出）
        reg signed [15:0] d;
        reg signed [15:0] r;
        reg [7:0]  s;
        reg [24:0] g;
        begin
            d = $signed({8'b0, c}) - $signed({8'b0, yy});  // -255..+255
            // 饱和度 x1.4375 = 1 + 0.5 - 0.0625（纯移位加法，不占 DSP）
            d = d + (d >>> 1) - (d >>> 4);
            r = $signed({8'b0, yy}) + d;
            if (r < 0)             s = 8'd0;
            else if (r > 16'sd255) s = 8'd255;
            else                   s = r[7:0];

            g = s * dg;                                    // 数字增益（dg 最大 640 -> 需 25bit）
            if (g > 25'd65280)
                enh_sat = 8'd255;
            else
                enh_sat = g[15:8];
        end
    endfunction

    // -------------------------------------------------------------------------
    // 对比度增强（独立后级，作用于最终显示像素，不影响自动曝光的亮度闭环）
    //   out = clamp( (c * GAIN) - BP )：先按 1.125x 拉伸，再去掉 16 黑位，
    //   把暗部压黑、中间调拉开，画面更通透、不发灰雾。
    // -------------------------------------------------------------------------
    localparam [15:0] CONTRAST_GAIN = 16'd288;  // 1.125x 拉伸（配合 -16 黑位，基本不挪动中间调）
    localparam [7:0]  CONTRAST_BP   = 8'd16;    // 黑位（去雾）
    function [7:0] enh_ct;
        input [7:0] c;
        reg [23:0] x;            // 24bit：c*288 最大 73440，必须 >=17bit，否则溢出回绕
        reg signed [24:0] v;
        begin
            x = {16'd0, c} * CONTRAST_GAIN;              // c * GAIN（24bit 不溢出）
            v = $signed({9'd0, x[23:8]}) - $signed({17'd0, CONTRAST_BP}); // (x>>8)-BP
            if (v < 25'sd0)        enh_ct = 8'd0;        // 负 -> 0
            else if (v > 25'sd255) enh_ct = 8'd255;
            else                   enh_ct = v[7:0];
        end
    endfunction

    // -------------------------------------------------------------------------
    // 白平衡：固定增益（已按"桌面实景"实测标定，无需再按键标定）
    //   对 FPGA 屏幕实拍做逐像素统计：屏幕绿背景 R/G≈0.90、B/G≈0.95，
    //   即红偏低约 10%、蓝略低 5%，画面因此发绿/发冷（灯管发蓝也是同一原因：
    //   红不足）。据此把 R 再抬高、B 略抬，使中性面还原为中性偏暖（符合现场暖光）。
    //   全链路（WB增益 -> gamma0.7 -> 饱和度x1.4375 -> 对比度）用 Python 精确建模
    //   逆解验证：增益弹性约 1.4x，故 R 288->316(+9.7%)、B 263->274(+4.2%) 即可中和。
    //   R/B 每 ±1 LSB 约影响 0.39% 色偏。
    //   （KEY1+KEY2 一键标定功能仍保留，但出厂值已正确，正常演示无需按。）
    // -------------------------------------------------------------------------
    localparam [15:0] WB_R_DEF = 16'd316;   // 1.234x（桌面实景：红偏低约 10%）
    localparam [15:0] WB_G_DEF = 16'd256;   // 1.000x（绿通道为基准）
    localparam [15:0] WB_B_DEF = 16'd274;   // 1.070x（蓝略低 5%，略抬）
    localparam [15:0] WB_GAIN_MIN = 16'd128;  // 0.5x（增益下限钳位）
    localparam [15:0] WB_GAIN_MAX = 16'd512;  // 2.0x（增益上限钳位）

    // 中央 ROI 累加（约 60% 画面）：x∈[256,1024)  y∈[144,576)
    reg  [31:0] acc_r, acc_g, acc_b;
    reg  [19:0] acc_cnt;
    reg  [31:0] lat_r, lat_g, lat_b;
    reg  [19:0] lat_cnt;
    wire wb_in_roi = (S_x >= 12'd256) && (S_x < 12'd1024) &&
                     (S_y >= 12'd144) && (S_y < 12'd576);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            acc_r<=32'd0; acc_g<=32'd0; acc_b<=32'd0; acc_cnt<=20'd0;
            lat_r<=32'd0; lat_g<=32'd0; lat_b<=32'd0; lat_cnt<=20'd0;
        end else begin
            if (I_video_user) begin
                // 帧开始：锁存上一帧统计，清零本帧累加器
                lat_r<=acc_r; lat_g<=acc_g; lat_b<=acc_b; lat_cnt<=acc_cnt;
                acc_r<=32'd0; acc_g<=32'd0; acc_b<=32'd0; acc_cnt<=20'd0;
            end else if (I_video_de && wb_in_roi) begin
                acc_r <= acc_r + {24'd0, I_video_rd_data[23:16]};
                acc_g <= acc_g + {24'd0, I_video_rd_data[15:8]};
                acc_b <= acc_b + {24'd0, I_video_rd_data[7:0]};
                acc_cnt <= acc_cnt + 20'd1;
            end
        end
    end

    // 三通道白平衡增益（Q8，默认取实拍标定值）。
    reg [15:0] wb_r_gain, wb_g_gain, wb_b_gain;

    // 标定状态机：移位减法做无符号除法 num/den -> 商（Q8 增益）
    //   只校色不校亮度：以 G 为基准，gain_ch = 256 * sum_g / sum_ch
    //   -> G 通道算出来恒为 256(1.0)，画面亮度完全不变，只把色偏拉正。
    reg [1:0]  cal_state;   // 0 idle, 1 dividing, 2 done
    reg [1:0]  cal_ch;      // 0=R 1=G 2=B
    reg [5:0]  cal_cnt;     // 40..1 除法迭代计数
    reg        cal_busy;
    reg [39:0] dv_num0, dv_num, dv_den, dv_rem, dv_quo;
    reg        wb_cal_d, wb_rst_d;
    wire       wb_cal_p = I_wb_cal && !wb_cal_d;
    wire       wb_rst_p = I_wb_rst && !wb_rst_d;
    wire [15:0] cal_gain_clamped =
                   (dv_quo[15:0] > WB_GAIN_MAX) ? WB_GAIN_MAX :
                   (dv_quo[15:0] < WB_GAIN_MIN) ? WB_GAIN_MIN : dv_quo[15:0];

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            wb_r_gain<=WB_R_DEF; wb_g_gain<=WB_G_DEF; wb_b_gain<=WB_B_DEF;
            cal_state<=2'd0; cal_ch<=2'd0; cal_cnt<=6'd0; cal_busy<=1'b0;
            dv_num0<=40'd0; dv_num<=40'd0; dv_den<=40'd0; dv_rem<=40'd0; dv_quo<=40'd0;
            wb_cal_d<=1'b0; wb_rst_d<=1'b0;
        end else begin
            wb_cal_d <= I_wb_cal;
            wb_rst_d <= I_wb_rst;

            if (wb_rst_p) begin
                // 恢复出厂固定增益（实测标定值）
                wb_r_gain<=WB_R_DEF; wb_g_gain<=WB_G_DEF; wb_b_gain<=WB_B_DEF;
                cal_state<=2'd0; cal_busy<=1'b0;
            end else if (wb_cal_p && !cal_busy) begin
                // 覆盖检查：中央 ROI 需有足够样本，且绿通道不能过暗（防除零/误标定）
                if (lat_cnt >= 20'd50000 && lat_g >= 32'd1000) begin
                    cal_state <= 2'd1;
                    cal_busy  <= 1'b1;
                    cal_ch    <= 2'd0;
                    cal_cnt   <= 6'd40;
                    dv_num0   <= {8'd0, lat_g} << 8;   // sum_g * 256（三通道共用分子）
                    dv_num    <= {8'd0, lat_g} << 8;
                    dv_den    <= {8'd0, lat_r};        // 分母 = 本通道累加和
                    dv_rem    <= 40'd0;
                    dv_quo    <= 40'd0;
                end
                // 样本不足则忽略本次请求，不改变已有增益
            end else if (cal_state == 2'd1) begin
                // 恢复余数除法：每拍处理 1 位（MSB 先入）
                dv_rem <= (dv_rem << 1) | dv_num[39];
                dv_num <= dv_num << 1;
                if (dv_rem >= dv_den) begin
                    dv_rem <= dv_rem - dv_den;
                    dv_quo <= (dv_quo << 1) | 40'd1;
                end else begin
                    dv_quo <= (dv_quo << 1);
                end
                if (cal_cnt == 6'd1) begin
                    // 本通道除法完成，锁存钳位后的增益
                    case (cal_ch)
                        2'd0: wb_r_gain <= cal_gain_clamped;
                        2'd1: wb_g_gain <= cal_gain_clamped;
                        2'd2: wb_b_gain <= cal_gain_clamped;
                    endcase
                    if (cal_ch != 2'd2) begin
                        cal_ch  <= cal_ch + 2'd1;
                        cal_cnt <= 6'd40;
                        dv_rem  <= 40'd0;
                        dv_quo  <= 40'd0;
                        dv_num  <= dv_num0;
                        dv_den  <= (cal_ch == 2'd0) ? {8'd0, lat_g} : {8'd0, lat_b};
                    end else begin
                        cal_state <= 2'd2;
                        cal_busy  <= 1'b0;
                    end
                end else begin
                    cal_cnt <= cal_cnt - 6'd1;
                end
            end else if (cal_state == 2'd2) begin
                cal_state <= 2'd0;
            end
        end
    end

    // 白平衡应用（Q8 定点）：默认 1.0（旁路）；标定后由硬件写入 wb_*_gain。
    // !! 位宽陷阱（曾导致"灯管整片变绿"）!!
    //   8bit 像素 x 最大 512 的增益 -> 乘积最大 130560，需要 17bit 才放得下。
    //   若只取乘积的 [15:8]（相当于丢弃 [23:16] 高位），raw >= 208 的亮像素
    //   就会回绕成很小的值：raw 255 x 316 = 80580 -> 取低 16bit 后 >>8 = 58，
    //   而绿通道 255 x 256 = 65280（恰好没溢出）仍为 255，于是亮部被压成纯绿。
    //   正确做法：先看完整乘积 [23:8] 是否超过 255，超了才钳位，否则取 [15:8]。
    wire [23:0] wb_r_x = ({16'd0, I_video_rd_data[23:16]}) * wb_r_gain;
    wire [23:0] wb_g_x = ({16'd0, I_video_rd_data[15:8]})  * wb_g_gain;
    wire [23:0] wb_b_x = ({16'd0, I_video_rd_data[7:0]})   * wb_b_gain;
    wire [15:0] wb_r_s = wb_r_x[23:8];   // 真实商（>>8），16bit 足够
    wire [15:0] wb_g_s = wb_g_x[23:8];
    wire [15:0] wb_b_s = wb_b_x[23:8];
    wire [7:0]  wb_r   = (wb_r_s > 16'd255) ? 8'd255 : wb_r_s[7:0];
    wire [7:0]  wb_g   = (wb_g_s > 16'd255) ? 8'd255 : wb_g_s[7:0];
    wire [7:0]  wb_b   = (wb_b_s > 16'd255) ? 8'd255 : wb_b_s[7:0];
    wire [23:0] wb_rgb = { wb_r, wb_g, wb_b };

    wire [7:0]  en_r  = enh_gamma07(wb_r);
    wire [7:0]  en_g  = enh_gamma07(wb_g);
    wire [7:0]  en_b  = enh_gamma07(wb_b);
    // 注意：乘法必须用 16bit 常量，否则 8bit 上下文会被截断
    wire [15:0] en_y16 = en_r * 16'd77 + en_g * 16'd150 + en_b * 16'd29;
    wire [7:0]  en_y   = en_y16[15:8];
    wire [23:0] enh0   = { enh_sat(en_r, en_y, dig_gain),
                           enh_sat(en_g, en_y, dig_gain),
                           enh_sat(en_b, en_y, dig_gain) };

    wire [15:0] gray16 = enh0[23:16] * 16'd77 +
                         enh0[15:8]  * 16'd150 +
                         enh0[7:0]   * 16'd29;
    wire [7:0]  gray0  = gray16[15:8];

    // -------------------------------------------------------------------------
    // 自动曝光（AEC）：逐帧统计中央 ROI 的输出亮度，调节 dig_gain 使平均亮度
    //   收敛到 AEC_TARGET_Y。这样"暗房间"和"亮赛场"都能自动通透，不会像固定
    //   增益那样一换场地就过曝/发灰。
    //   ROI 与白平衡统计同一块：x∈[256,1024) y∈[144,576) -> 768*432 = 331776 像素
    //   每帧只在帧起始更新一次；偏差大时粗调 16，接近后精调 3，死区 ±2 LSB。
    // -------------------------------------------------------------------------
    // 目标平均亮度 128 -> 116：现场反馈"有点曝光"，下调后画面更沉、亮部不再一片死白；
    //   同时把提亮上限从 2.5x 收到 2.0x，避免高增益把亮部削平（削平后颜色信息就没了）
    localparam [31:0] AEC_TARGET_SUM = 32'd38486016; // 331776 * 116（目标平均亮度 116）
    localparam [31:0] AEC_BIG_DIFF   = 32'd5308416;  // 331776 * 16（偏差 >16 LSB 认为大）
    localparam [31:0] AEC_DEAD_DIFF  = 32'd663552;   // 331776 * 2 （死区 ±2 LSB）
    localparam [15:0] AEC_GAIN_MIN   = 16'd128;      // 0.50x
    localparam [15:0] AEC_GAIN_MAX   = 16'd512;      // 2.00x（原 2.5x，收小以减轻亮部削顶）

    reg  [31:0] acc_y, lat_y;
    reg  [19:0] acc_y_cnt, lat_y_cnt;
    reg         vuser_d;
    wire        vuser_p = I_video_user & ~vuser_d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            acc_y<=32'd0; lat_y<=32'd0; acc_y_cnt<=20'd0; lat_y_cnt<=20'd0; vuser_d<=1'b0;
        end else begin
            vuser_d <= I_video_user;
            if (vuser_p) begin
                // 帧起始：锁存刚统计完的一帧，清零本帧累加器
                lat_y <= acc_y; lat_y_cnt <= acc_y_cnt;
                acc_y <= 32'd0; acc_y_cnt <= 20'd0;
            end else if (I_video_de && wb_in_roi) begin
                acc_y     <= acc_y + {24'd0, gray0};
                acc_y_cnt <= acc_y_cnt + 20'd1;
            end
        end
    end

    wire aec_valid      = (lat_y_cnt >= 20'd200000);
    wire aec_too_bright = (lat_y > AEC_TARGET_SUM);
    wire [31:0] aec_diff = aec_too_bright ? (lat_y - AEC_TARGET_SUM)
                                          : (AEC_TARGET_SUM - lat_y);

    // -------------------------------------------------------------------------
    // 模式 6（运动检测）锁定自动曝光
    //   帧差法比的是"同一块亮度有没有变"，对整帧亮度变化零容忍：AEC 每帧改
    //   dig_gain 会让全屏所有块一起变，判据必然满屏红 + 蜂鸣器长鸣。
    //   v3 曾用"增益归一化"抵消，但归一化把差值放大 G^2/256 倍、阈值却是常数，
    //   等效阈值 = TH*65536/(G_now*G_prev)：gain=128 时高达 48 LSB（几乎不响应），
    //   gain=512 时只剩 3 LSB（噪声直接轰穿）—— 同一个式子两头都错。
    //   => 检测期间直接锁死增益：切进模式 6 后先让 AEC 收敛 16 帧，然后冻结。
    //   冻结后再等 2 帧才放开判决：gray_ram 里还留着"锁定前那一帧"的块值。
    // -------------------------------------------------------------------------
    wire        m6_mode = (I_mode == 3'd6);
    reg  [5:0]  m6_cnt;      // 进入模式 6 后已过的帧数
    reg         m6_frz;      // 1 = 曝光已锁定

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            m6_cnt <= 6'd0; m6_frz <= 1'b0;
        end else if (!m6_mode) begin
            m6_cnt <= 6'd0; m6_frz <= 1'b0;
        end else if (vuser_p) begin
            if (m6_cnt != 6'd63) m6_cnt <= m6_cnt + 6'd1;
            if (m6_cnt == 6'd16) m6_frz <= 1'b1;
        end
    end

    wire aec_freeze = m6_frz;
    wire m6_ready   = m6_frz && (m6_cnt >= 6'd18);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            dig_gain <= 16'd320;              // 1.25x 起步（原 1.5x），上电不会先过曝一下
        end else if (vuser_p && aec_valid && !cal_busy && !aec_freeze) begin
            if (aec_diff > AEC_BIG_DIFF) begin
                // 粗调 16（6.25%）
                if (aec_too_bright)
                    dig_gain <= (dig_gain > (AEC_GAIN_MIN + 16'd16)) ? (dig_gain - 16'd16) : AEC_GAIN_MIN;
                else
                    dig_gain <= (dig_gain < (AEC_GAIN_MAX - 16'd16)) ? (dig_gain + 16'd16) : AEC_GAIN_MAX;
            end else if (aec_diff > AEC_DEAD_DIFF) begin
                // 精调 3（1.17%）
                if (aec_too_bright)
                    dig_gain <= (dig_gain > (AEC_GAIN_MIN + 16'd3)) ? (dig_gain - 16'd3) : AEC_GAIN_MIN;
                else
                    dig_gain <= (dig_gain < (AEC_GAIN_MAX - 16'd3)) ? (dig_gain + 16'd3) : AEC_GAIN_MAX;
            end
        end
    end

    wire [7:0] w00,w01,w02,w10,w11,w12,w20,w21,w22;
    window3x3_gray u_w3 (
        .I_clk(I_clk), .I_pix_en(I_video_de), .I_gray(gray0),
        .p00(w00),.p01(w01),.p02(w02),
        .p10(w10),.p11(w11),.p12(w12),
        .p20(w20),.p21(w21),.p22(w22)
    );

    // -------------------------------------------------------------------------
    // 模式 6：运动目标检测（帧差法）
    //   把 1280x720 按 8x8 归并成 160x90 个块，块值取块内 64 像素灰度均值；
    //   与上一帧同块均值比较，差 > 13 LSB 判定该块有运动。
    //   检测期间 AEC 增益被锁死（见上面的"模式 6 锁定自动曝光"），否则整帧亮度
    //   平移会被当成全屏运动 —— 这是 v1~v3 反复出问题的根源。
    //   实现要点见 img_proc/motion_detect.v 头部注释（单缓冲帧差、不用 DSP、
    //   块内平均把噪声压 8 倍所以不需要再做 3x3 滤波）。
    //   原来这里的 median3x3 + defect_detect 一并去掉：缺陷检测不属于本赛题
    //   评分项，且两者都在 clk1(74.25MHz) 关键路径上，去掉可减轻时序压力。
    // -------------------------------------------------------------------------
    wire        mv_motion;      // 本块有运动（已对齐 pd1/pd2）
    wire [15:0] mv_cnt;         // 本帧运动块数（实时）
    wire [15:0] mv_cnt_lat;     // 上一帧运动块数（帧起始锁存，供 OSD 稳定显示）
    wire        mv_guard;       // 上一帧判为"全局突变"（开灯/关灯/曝光跳变）
    wire [7:0]  mv_maxd;        // 上一帧最大块差（诊断）
    wire [15:0] mv_raw;         // 上一帧超阈值块数（滤波前，诊断）

    motion_detect #(
        .H_OFFSET(H_OFFSET), .V_OFFSET(V_OFFSET),
        .IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT),
        // 2026-10-31：阈值 18 -> 24。夜景静止实测 9~50 块随机乱跳（尖峰越过了
        // 24 的门槛误响），块差尾部比白天厚；手挥块差 40~150，24 仍有 2 倍余量。
        .THRESH(8'd24),
        .CLIP_GRD(8'd250),      // 块均值 >=250 视为削顶：饱和后差值无意义，不参与判定
        .GUARD_CNT(16'd2160)    // 3600 块的 60%：判为全局突变（开灯/关灯），不显示不报警
    ) u_mv (
        .I_clk(I_clk), .I_rst_n(I_rst_n),
        .I_pix_en(S_window_active), .I_frame(I_video_user),
        .I_x(S_x), .I_y(S_y), .I_gray(gray0),
        .I_en(m6_ready),        // 1 = 模式 6 且曝光已锁定，允许判决
        .O_motion(mv_motion),
        .O_cnt(mv_cnt), .O_cnt_lat(mv_cnt_lat),
        .O_guard(mv_guard),
        .O_maxd(mv_maxd), .O_raw(mv_raw),
        .O_ready()
    );

    // 「有运动」判定：不再是"有 1 块就算"，而是要求至少 24 块
    //   现场实测（v7 上板录屏逐帧统计，16x16 网格 3600 块）：
    //     静止时残余     0 ~ 7 块
    //     手从镜头前划过 108 ~ 263 块
    //   两档差一个数量级，门槛取 24 既有 3 倍余量、又不会漏掉较弱的一次挥手。
    //   这条同时决定：蜂鸣器响不响、OSD 显示 BUSY 还是 IDLE。
    //   全局突变（开灯/关灯/曝光跳变）时不算运动，避免满屏红 + 蜂鸣器长鸣。
    wire mv_present = (mv_cnt_lat >= 16'd24) && !mv_guard;

    assign O_mv_present = mv_present;
    assign O_mv_cnt     = mv_cnt_lat;


    wire [23:0] algo_rgb;
    img_algo u_algo (
        .I_mode(I_mode), .I_rgb(enh0),
        .p00(w00),.p01(w01),.p02(w02),
        .p10(w10),.p11(w11),.p12(w12),
        .p20(w20),.p21(w21),.p22(w22),
        .O_rgb(algo_rgb)
    );

    // 模式 6（MOTION）：运动块涂红，静止块显示灰度
    //   灰度延 2 拍，与 mv_motion 的 2 拍延迟对齐（差 1~2 像素在 8x8 块内无感）
    reg [7:0] mv_g1, mv_g2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin mv_g1 <= 8'd0; mv_g2 <= 8'd0; end
        else begin mv_g1 <= gray0; mv_g2 <= mv_g1; end
    end
    wire [23:0] motion_disp = mv_motion ? 24'hFF2020 : {mv_g2, mv_g2, mv_g2};

    // 模式 0..5 走原算法；模式 6 走运动可视化
    wire [23:0] proc0 = (I_mode == 3'd6) ? motion_disp : algo_rgb;

    reg [23:0] pd1, pd2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin pd1<=0; pd2<=0; end
        else begin pd1<=proc0; pd2<=pd1; end
    end

    // -------------------------------------------------------------------------
    // 颜色目标检测（外接框 + 计数，锁存供 OSD 使用）
    //   ★ 2026-10-28 15:20 判色源改为「传感器原始像素」I_video_rd_data。
    //     原因（用 Python 精确复刻整条链后量出来的）：
    //       ① 白平衡固定增益 R=1.234x 会让任何中性面变成 R/G≈1.25 的品红，
    //          墙壁/白纸的"纯度"被凭空抬到 20% 左右 —— 判色和显示都被它带偏；
    //       ② 后级 AEC 增益（最高 2x）会先把饱和度削顶到 255 再放大，
    //          中性面只要输入 >=150 就整块变纯白(255,255,255)，颜色信息全丢；
    //          实测天蓝(70,150,225) 显示成 (121,238,255) -> 判不出蓝。
    //     用原始像素判色：不削顶、无偏色增益，纯度只反映物体本身颜色。
    //     屏幕显示完全不变（enh0 那条路一行未改），判色只是"另取一路"。
    // -------------------------------------------------------------------------
    wire [11:0] bx_min, bx_max, by_min, by_max;
    wire [15:0] obj_cnt;
    color_obj_detect u_detect (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_pix_en(I_video_de),
        .I_user(I_video_user), .I_x(S_x), .I_y(S_y),
        .I_rgb(I_video_rd_data), .I_target_sel(I_target_sel),
        .O_bx_min(bx_min), .O_bx_max(bx_max),
        .O_by_min(by_min), .O_by_max(by_max),
        .O_count(obj_cnt), .O_match()
    );

    // -------------------------------------------------------------------------
    // 帧计数（用于 OSD 延迟/帧率可视化）
    // -------------------------------------------------------------------------
    reg [15:0] frame_cnt;
    reg        user_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin frame_cnt<=0; user_d<=0; end
        else begin
            user_d <= I_video_user;
            if(I_video_user && !user_d) frame_cnt <= frame_cnt + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // 端到端延迟测量（2026-10-28 新增，对应赛题"处理延迟可视化"）
    //   方法：
    //     ① 顶层把摄像头帧起始 tuser 在采集时钟域展宽到约 16 拍，避免 1 拍
    //        脉冲跨到 74.25MHz 像素域时被同步器漏采；本模块收到 I_cam_fs；
    //     ② 3 级同步 + 上升沿检测，统计两次采集帧起始之间的像素时钟数，
    //        即实测帧周期（74.25MHz 下约 1,237,500 拍 = 16.67ms）；
    //     ③ 本工程是 DDR2 乒乓双缓冲、写一帧读一帧，显示延迟恒为 1 帧，
    //        故 延迟 = 1 x 实测帧周期，单位 0.1ms。
    //   为什么不直接"采集时间戳 - 显示时间戳"：采集与显示同频并行、相位固定
    //   超前约一帧，两时间戳相减恒接近 0，是无意义的数；帧周期 x 缓冲级数
    //   才是真实延迟，且数字随实际场频变化（16.6~16.7），可证明测量电路在工作。
    //   乘法用移位相加实现（x142 >> 20 ~= /7384，即 74.25MHz 下每 0.1ms 的拍数），
    //   不用 DSP、不用除法器，逻辑深度浅。
    // -------------------------------------------------------------------------
    reg  [2:0]  cam_fs_sync;
    reg  [23:0] cam_fs_t0, cam_fs_t1;
    reg  [23:0] freerun;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            cam_fs_sync <= 3'd0; cam_fs_t0 <= 24'd0; cam_fs_t1 <= 24'd0; freerun <= 24'd0;
        end else begin
            cam_fs_sync <= {cam_fs_sync[1:0], I_cam_fs};
            freerun     <= freerun + 24'd1;
            if(cam_fs_sync[2] && !cam_fs_sync[1]) begin
                cam_fs_t0 <= cam_fs_t1;      // 上一次采集帧起始
                cam_fs_t1 <= freerun;        // 本次采集帧起始
            end
        end
    end

    wire [23:0] frm_period = cam_fs_t1 - cam_fs_t0;      // 实测帧周期（像素时钟数）
    wire [31:0] per_x142   = ({8'd0,frm_period} << 7) + ({8'd0,frm_period} << 3)
                           + ({8'd0,frm_period} << 2) + ({8'd0,frm_period} << 1);

    reg  [11:0] dly_hold;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) dly_hold <= 12'd167;
        else if (per_x142[31:20] >= 12'd10 && per_x142[31:20] <= 12'd500)
            dly_hold <= per_x142[31:20];     // 只接受 1.0ms~50.0ms，防丢帧抖动
    end

    // ---- 十进制拆位：目标框左上角 X/Y 与延迟，每帧启动一次，12 拍串行完成 ----
    wire [3:0] xd3, xd2, xd1, xd0;
    wire [3:0] yd2, yd1, yd0;
    wire [3:0] dd2, dd1, dd0;
    wire       busy_x, busy_y, busy_d;
    wire       bcd_start = I_video_user & ~user_d;       // 每帧一拍

    bin2bcd u_bcd_x (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_start(bcd_start), .I_bin(bx_min),
        .O_d3(xd3), .O_d2(xd2), .O_d1(xd1), .O_d0(xd0), .O_busy(busy_x));

    bin2bcd u_bcd_y (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_start(bcd_start), .I_bin(by_min),
        .O_d3(), .O_d2(yd2), .O_d1(yd1), .O_d0(yd0), .O_busy(busy_y));

    bin2bcd u_bcd_d (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_start(bcd_start), .I_bin(dly_hold),
        .O_d3(), .O_d2(dd2), .O_d1(dd1), .O_d0(dd0), .O_busy(busy_d));

    // 运动块数转十进制（OSD 显示 "MV=1234"）
    //   原来显示十六进制，现场无法一眼判断量级；这里复用 bin2bcd（12 拍串行，每帧一次）。
    //   3600 块 < 4096，12 位足够，高位直接补零。
    wire [3:0] md3, md2, md1, md0;
    wire       mbusy;
    bin2bcd u_bcd_m (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_start(bcd_start), .I_bin(mv_cnt_lat[11:0]),
        .O_d3(md3), .O_d2(md2), .O_d1(md1), .O_d0(md0), .O_busy(mbusy)
    );

    // ---- 2026-10-31：最大块差转十进制（模式6 诊断行 "D=nnn"）----
    //   与 MV 那一路并行走，同一次 bcd_start 触发，互不干扰。
    //   注意：xd*/u_bcd_x 已被"目标框 X 坐标"占用，这里另起 qd*/u_bcd_q。
    wire [3:0] qd2, qd1, qd0;
    wire       qbusy;
    bin2bcd u_bcd_q (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_start(bcd_start), .I_bin({4'd0, mv_maxd}),
        .O_d3(), .O_d2(qd2), .O_d1(qd1), .O_d0(qd0), .O_busy(qbusy)
    );

    // ---- 2026-10-31：中央 ROI 平均原始 RGB（OSD 显示 "M:RRGGBB"，模式5 调判色用）----
    //   lat_r/lat_g/lat_b 是上一帧中央 60% ROI（x 256..1023, y 144..575 = 331776 像素）
    //   的累加和，除以 331776 即均值。这里不做除法，用 sum*3>>20 近似
    //   （3/2^20 = 2.86e-6，1/331776 = 3.01e-6，差 5%，且最大值 255*0.95 不会溢出 8bit）。
    //   比例判断只需要通道之间的比值，5% 的公共缩放无影响。
    wire [31:0] mn_r = (lat_r << 1) + lat_r;
    wire [31:0] mn_g = (lat_g << 1) + lat_g;
    wire [31:0] mn_b = (lat_b << 1) + lat_b;
    wire [7:0]  pxm_r = mn_r[27:20];
    wire [7:0]  pxm_g = mn_g[27:20];
    wire [7:0]  pxm_b = mn_b[27:20];

    // -------------------------------------------------------------------------
    // OSD 叠加
    // -------------------------------------------------------------------------
    wire       osd_on0;
    wire [23:0] osd_col0;
    osd_overlay u_osd (
        .I_x(S_x), .I_y(S_y), .I_osd_en(I_osd_en), .I_mode(I_mode),
        .I_frame(frame_cnt), .I_count(obj_cnt), .I_target_sel(I_target_sel),
        .I_bx_min(bx_min), .I_bx_max(bx_max),
        .I_by_min(by_min), .I_by_max(by_max),
        .I_mv_present(mv_present), .I_mv_cnt(mv_cnt_lat),
        .I_mv_d3(md3), .I_mv_d2(md2), .I_mv_d1(md1), .I_mv_d0(md0),
        .I_xd3(xd3), .I_xd2(xd2), .I_xd1(xd1), .I_xd0(xd0),
        .I_yd2(yd2), .I_yd1(yd1), .I_yd0(yd0),
        .I_dl2(dd2), .I_dl1(dd1), .I_dl0(dd0),
        .I_pxm_r(pxm_r), .I_pxm_g(pxm_g), .I_pxm_b(pxm_b),
        .I_mxd2(qd2), .I_mxd1(qd1), .I_mxd0(qd0), .I_raw(mv_raw),
        .O_on(osd_on0), .O_color(osd_col0)
    );

    reg        osd_on1, osd_on2;
    reg [23:0] osd_col1, osd_col2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin osd_on1<=0; osd_on2<=0; osd_col1<=0; osd_col2<=0; end
        else begin osd_on1<=osd_on0; osd_on2<=osd_on1;
               osd_col1<=osd_col0; osd_col2<=osd_col1; end
    end

    // 对比度增强（独立后级，仅作用于图像本体，不碰 OSD 文字/框，保持叠加层清晰）
    wire [23:0] pd2_ct = { enh_ct(pd2[23:16]),
                           enh_ct(pd2[15:8]),
                           enh_ct(pd2[7:0]) };

    // -------------------------------------------------------------------------
    // 同步/数据 2 拍延迟（与原模块一致）
    // -------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0; S_video_vsync_2d <= 1'b0;
            S_video_hsync_1d <= 1'b0; S_video_hsync_2d <= 1'b0;
            S_video_de_1d    <= 1'b0; S_video_de_2d    <= 1'b0;
            S_window_active_1d <= 1'b0; S_window_active_2d <= 1'b0;
            O_hdmi_vsync <= 1'b0; O_hdmi_hsync <= 1'b0;
            O_hdmi_de    <= 1'b0; O_hdmi_data  <= 24'd0;
        end else begin
            S_video_vsync_1d <= I_video_vsync; S_video_vsync_2d <= S_video_vsync_1d;
            O_hdmi_vsync <= S_video_vsync_2d;
            S_video_hsync_1d <= I_video_hsync; S_video_hsync_2d <= S_video_hsync_1d;
            O_hdmi_hsync <= S_video_hsync_2d;
            S_video_de_1d <= I_video_de; S_video_de_2d <= S_video_de_1d;
            S_window_active_1d <= S_window_active; S_window_active_2d <= S_window_active_1d;
            O_hdmi_de <= S_video_de_2d;
            S_video_data_1d <= I_video_rd_data; S_video_data_2d <= S_video_data_1d;

            O_hdmi_data <= S_window_active_2d ? (osd_on2 ? osd_col2 : pd2_ct) : 24'd0;
        end
    end

endmodule


// =============================================================================
// 以下 4 个模块原为独立文件，因 TD 工程文件清单维护问题，内联到本文件一并编译；
// 功能与独立版本完全一致（median3x3 / defect_detect / uart_tx / defect_alarm）。
// =============================================================================

// ---- img_proc/median3x3.v ----
// =============================================================================
// median3x3.v  --  3x3 中值滤波（噪声滤波）
//
// 用一个 3x3 窗口产生"去噪后的中心像素"。为省资源、降组合深度，采用
// "行列分离"实现：先对各⾏取 3 点中值，再对 3 个⾏中值取中值。
// 对工业表面检测里最常见的椒盐/孤立噪声（单个亮/暗噪点）能有效滤除，
// 且只用若干加法/比较，不占 DSP、无除法。
//
// med3(a,b,c) = a + b + c − min(a,b,c) − max(a,b,c)   （三者之和减极值，即中值）
// 纯组合、0 额外延迟，与窗口同拍输出，便于与 Sobel / 缺陷检测对齐。
// =============================================================================
module median3x3 (
    input  wire [7:0] p00, p01, p02,
    input  wire [7:0] p10, p11, p12,
    input  wire [7:0] p20, p21, p22,
    output wire [7:0] O_med
);

    // 三点中值：和 - 最小值 - 最大值（结果必为三者之一，<=255）
    function [7:0] med3;
        input [7:0] a;
        input [7:0] b;
        input [7:0] c;
        reg  [9:0] s;
        reg  [7:0] mn, mx;
        begin
            s  = {2'd0, a} + {2'd0, b} + {2'd0, c};              // 最大 765
            mn = (a <= b) ? ((a <= c) ? a : c) : ((b <= c) ? b : c);
            mx = (a >= b) ? ((a >= c) ? a : c) : ((b >= c) ? b : c);
            med3 = s[7:0] - mn - mx;                             // s-mn-mx = 中值（高位为 0）
        end
    endfunction

    wire [7:0] r0 = med3(p00, p01, p02);       // 上一行 行中值
    wire [7:0] r1 = med3(p10, p11, p12);       // 当前行 行中值
    wire [7:0] r2 = med3(p20, p21, p22);       // 下一行 行中值

    assign O_med = med3(r0, r1, r2);           // 三行中值再取中值

endmodule


// ---- img_proc/defect_detect.v ----
// =============================================================================
// defect_detect.v  --  工件表面缺陷检测（划痕 / 凹坑 / 缺角）
//
// 输入：3x3 灰度窗口 p00..p22（来自去噪前/后的同一窗口）与去噪中心像素 med。
// 思路（全部为比较/加减，无除法、不占 DSP，纯流水线）：
//   1) 局部背景 bg = 8 邻域均值（sum8>>3）。
//   2) 凹坑(pit)：中心比"上下左右 4 个邻居"都暗（严格局部极小）—— 暗色小坑/黑点；
//      这样物体边缘的暗侧不会被误判成凹坑。
//   3) 划痕(scratch)：中心同时比"左右"或"上下"两个邻居暗 —— 细长的暗线。
//        · med + T_SCR < p10 且 med + T_SCR < p12  -> 竖向暗划痕
//        · med + T_SCR < p01 且 med + T_SCR < p21  -> 横向暗划痕
//   4) 缺角(corner)：Sobel 横向、纵向梯度同时很大 —— 两条边交汇/缺口。
//        · agx > T_COR 且 agy > T_COR
//   优先级：凹坑 > 划痕 > 缺角（同一像素只归一类）。
//
// 输出：
//   O_defect      像素级"是缺陷"标志（供显示红色高亮）
//   O_type        1=划痕 2=凹坑 3=缺角 0=无（供统计/显示）
//   O_cnt_scratch / O_cnt_pit / O_cnt_corner  上一帧三类缺陷像素计数
//   O_present     上一帧是否存在缺陷（任一计数 >= MIN_AREA），供蜂鸣器/串口
//
// 说明：类型判定为"启发式"，具体阈值需按实际工件/光照现场标定（见文件顶部参数）。
// =============================================================================
module defect_detect (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_pix_en,
    input  wire        I_user,          // 帧起始（首个有效像素）
    input  wire [7:0]  p00, p01, p02,
    input  wire [7:0]  p10, p11, p12,
    input  wire [7:0]  p20, p21, p22,
    input  wire [7:0]  I_med,           // 去噪后的中心像素
    output reg         O_defect,
    output reg  [1:0]  O_type,
    output reg  [15:0] O_cnt_scratch,
    output reg  [15:0] O_cnt_pit,
    output reg  [15:0] O_cnt_corner,
    output wire        O_present
);

    // ---- 可调阈值（现场标定入口）----
    localparam [7:0]  T_PIT     = 8'd42;    // 凹坑：中心比周围暗多少
    localparam [7:0]  T_SCR     = 8'd38;    // 划痕：中心比左右/上下暗多少
    localparam [11:0] T_COR     = 12'd160;  // 缺角：横纵梯度阈值
    localparam [15:0] MIN_AREA  = 16'd80;   // 判定"有缺陷"的最小像素数（抗噪；薄划痕像素少，别设太大）

    // ---- Sobel 梯度（用原始窗口，噪声由 med 中心与均值背景抵消一部分）----
    wire signed [11:0] s00 = $signed({4'd0, p00});
    wire signed [11:0] s01 = $signed({4'd0, p01});
    wire signed [11:0] s02 = $signed({4'd0, p02});
    wire signed [11:0] s10 = $signed({4'd0, p10});
    wire signed [11:0] s12 = $signed({4'd0, p12});
    wire signed [11:0] s20 = $signed({4'd0, p20});
    wire signed [11:0] s21 = $signed({4'd0, p21});
    wire signed [11:0] s22 = $signed({4'd0, p22});

    wire signed [11:0] gx = (s20 + (s21<<1) + s22) - (s00 + (s01<<1) + s02);
    wire signed [11:0] gy = (s00 + (s10<<1) + s20) - (s02 + (s12<<1) + s22);
    wire [11:0] agx = gx[11] ? (~gx + 12'd1) : gx;
    wire [11:0] agy = gy[11] ? (~gy + 12'd1) : gy;

    // ---- 8 邻域均值背景 ----
    wire [10:0] sum8 = p00 + p01 + p02 + p10 + p12 + p20 + p21 + p22;  // <=2040
    wire [7:0]  bg   = sum8[10:3];                                     // /8

    // ---- 9bit 加宽比较 ----
    // 注意：判据用"原始中心 p11"而不是 I_med。3x3 中值会把 1~2px 的薄划痕一并抹掉，
    // 若用去噪值判划痕会漏检；故中值仅用于"显示去噪"，检测用原始像素，靠 MIN_AREA 抗噪。
    wire [7:0] ctr  = p11;
    wire [8:0] ctr9 = {1'b0, ctr};
    wire [8:0] bg9  = {1'b0, bg};
    wire [8:0] tp9  = {1'b0, T_PIT};
    wire [8:0] ts9  = {1'b0, T_SCR};
    wire [8:0] p10_9 = {1'b0, p10};
    wire [8:0] p12_9 = {1'b0, p12};
    wire [8:0] p01_9 = {1'b0, p01};
    wire [8:0] p21_9 = {1'b0, p21};

    // 凹坑：中心比"上下左右 4 个邻居"都暗（严格局部极小）→ 只在真正的小暗点触发，
    //       物体边缘的暗侧不会被误判成凹坑（这是避免蜂鸣器乱响的关键）。
    wire pit     = ((ctr9 + tp9) < p10_9) && ((ctr9 + tp9) < p12_9) &&
                   ((ctr9 + tp9) < p01_9) && ((ctr9 + tp9) < p21_9);
    // 划痕：中心同时比"左右"或"上下"两个邻居暗 → 1px 左右的细暗线（边缘不满足"两侧都亮"）。
    wire scratch = ((ctr9 + ts9) < p10_9 && (ctr9 + ts9) < p12_9) ||
                   ((ctr9 + ts9) < p01_9 && (ctr9 + ts9) < p21_9);
    wire corner  = (agx > T_COR) && (agy > T_COR);

    wire hit = pit | scratch | corner;

    always @(*) begin
        if (pit)           O_type = 2'd2;      // 凹坑
        else if (scratch)  O_type = 2'd1;      // 划痕
        else if (corner)   O_type = 2'd3;      // 缺角
        else               O_type = 2'd0;
        O_defect = hit;
    end

    // ---- 逐帧计数 ----
    reg [15:0] cur_s, cur_p, cur_c;
    reg        user_d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            user_d <= 1'b0;
            cur_s <= 16'd0; cur_p <= 16'd0; cur_c <= 16'd0;
            O_cnt_scratch <= 16'd0; O_cnt_pit <= 16'd0; O_cnt_corner <= 16'd0;
        end else begin
            user_d <= I_user;
            if (I_user && !user_d) begin
                // 帧首：锁存刚统计完的一帧
                O_cnt_scratch <= cur_s;
                O_cnt_pit    <= cur_p;
                O_cnt_corner <= cur_c;
                cur_s <= 16'd0; cur_p <= 16'd0; cur_c <= 16'd0;
            end else if (I_pix_en && hit) begin
                if (pit)          cur_p <= cur_p + 16'd1;
                else if (scratch) cur_s <= cur_s + 16'd1;
                else              cur_c <= cur_c + 16'd1;
            end
        end
    end

    assign O_present = (O_cnt_scratch >= MIN_AREA) ||
                       (O_cnt_pit     >= MIN_AREA) ||
                       (O_cnt_corner  >= MIN_AREA);

endmodule


// ---- uart_tx.v ----
// =============================================================================
// uart_tx.v  --  标准 UART 发送器（8N1，默认 115200，可在例化时改参数）
//   CLK_HZ : 输入时钟频率（默认 74.25MHz = 1280x720@60 像素时钟）
//   BAUD   : 波特率（默认 115200）
//   send   : 单拍脉冲，启动一次发送；data_in 在 send 拍被锁存
//   btn    : 波特率分频 = CLK_HZ/BAUD
// =============================================================================
module uart_tx #(
    parameter integer CLK_HZ = 74250000,
    parameter integer BAUD   = 115200
)(
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire [7:0] I_data,
    input  wire       I_send,
    output reg        O_txd,
    output reg        O_busy
);

    localparam integer DIV = (CLK_HZ / BAUD) < 1 ? 1 : (CLK_HZ / BAUD);

    reg [15:0] cnt;
    reg [2:0]  bitidx;
    reg [7:0]  sh;
    reg [1:0]  state;              // 0 idle, 1 start, 2 data, 3 stop

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_txd <= 1'b1; O_busy <= 1'b0;
            cnt <= 16'd0; bitidx <= 3'd0; sh <= 8'd0; state <= 2'd0;
        end else begin
            case (state)
                2'd0: begin
                    O_txd <= 1'b1; O_busy <= 1'b0;
                    if (I_send) begin
                        sh <= I_data; O_busy <= 1'b1;
                        cnt <= 16'd0; bitidx <= 3'd0; state <= 2'd1;
                    end
                end
                2'd1: begin                        // 起始位（低）
                    O_txd <= 1'b0;
                    if (cnt == DIV-1) begin cnt <= 16'd0; state <= 2'd2; bitidx <= 3'd0; end
                    else cnt <= cnt + 16'd1;
                end
                2'd2: begin                        // 8 数据位（LSB 先）
                    O_txd <= sh[bitidx];
                    if (cnt == DIV-1) begin
                        cnt <= 16'd0;
                        if (bitidx == 3'd7) state <= 2'd3;
                        else bitidx <= bitidx + 3'd1;
                    end else cnt <= cnt + 16'd1;
                end
                2'd3: begin                        // 停止位（高）
                    O_txd <= 1'b1;
                    if (cnt == DIV-1) begin cnt <= 16'd0; state <= 2'd0; O_busy <= 1'b0; end
                    else cnt <= cnt + 16'd1;
                end
            endcase
        end
    end

endmodule


// ---- defect_alarm.v ----
// =============================================================================
// defect_alarm.v  --  缺陷报警 + 串口状态上传
//
// 功能：
//   1) 蜂鸣器：检测到缺陷（I_present）时，产生约 2.7kHz 断续蜂鸣（响 0.25s / 停 0.25s），
//      输出低电平有效（板上 BUZZER 经三极管，低电平导通发声）。
//   2) 串口：每 0.5s 通过 UART(115200,8N1) 向上位机发送一行 ASCII 状态：
//         D<p>\n?  —— 具体格式见下行：
//         "D0,S0012,P0003,C0001\r\n"
//       D=0/1 是否有缺陷，S/P/C = 划痕/凹坑/缺角 的像素计数（4 位十六进制）。
//
// 全部在像素时钟（默认 74.25MHz）下运行，无需 CPU。
// =============================================================================
module defect_alarm #(
    parameter integer CLK_HZ = 74250000,
    parameter integer BAUD   = 115200
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_present,
    input  wire [15:0] I_cnt_scratch,
    input  wire [15:0] I_cnt_pit,
    input  wire [15:0] I_cnt_corner,
    output wire        O_buzzer,       // 低电平有效
    output wire        O_uart_txd
);

    // -------------------------------------------------------------------------
    // 蜂鸣器：2.7kHz 方波 + 0.25s 通断门控
    // -------------------------------------------------------------------------
    localparam [15:0] TONE_HALF = (CLK_HZ/2700)/2;   // 半周期≈13750
    localparam [24:0] GATE_CYC  = CLK_HZ/4;          // 0.25s

    reg        tone;
    reg [15:0] tone_cnt;
    reg        gate;
    reg [24:0] gate_cnt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            tone <= 1'b0; tone_cnt <= 16'd0;
            gate <= 1'b1; gate_cnt <= 25'd0;
        end else begin
            if (I_present) begin
                if (tone_cnt >= TONE_HALF-16'd1) begin tone_cnt <= 16'd0; tone <= ~tone; end
                else tone_cnt <= tone_cnt + 16'd1;
            end else begin
                tone <= 1'b0; tone_cnt <= 16'd0;
            end
            if (gate_cnt >= GATE_CYC-25'd1) begin gate_cnt <= 25'd0; gate <= ~gate; end
            else gate_cnt <= gate_cnt + 25'd1;
        end
    end

    // 低电平有效：present 且门控有效时，按 tone 断续拉低
    assign O_buzzer = (I_present && gate) ? ~tone : 1'b1;

    // -------------------------------------------------------------------------
    // 周期上报（0.5s 一次）
    // -------------------------------------------------------------------------
    localparam [26:0] RPORT_CYC = CLK_HZ/2;           // 0.5s
    localparam [5:0]  LEN       = 6'd22;              // 字符个数

    reg [26:0] rp_cnt;
    wire       rp_tick = (rp_cnt >= RPORT_CYC-27'd1);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) rp_cnt <= 27'd0;
        else rp_cnt <= rp_tick ? 27'd0 : rp_cnt + 27'd1;
    end

    // ---- 发送 FSM ----
    reg [1:0]  st;             // 0 idle 1 pulse 2 wait-busy 3 wait-done
    reg [5:0]  idx;
    reg        uart_send;
    reg [7:0]  uart_data;
    wire       uart_busy;

    reg        present_d;
    reg [15:0] s_d, p_d, c_d;

    // ---- 字节组装 ----
    function [7:0] hex2a;
        input [3:0] n;
        begin hex2a = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h37 + {4'd0, n}); end
    endfunction

    function [7:0] nib;
        input [15:0] v;
        input [2:0]  k;
        begin
            case (k)
                3'd0: nib = hex2a(v[3:0]);
                3'd1: nib = hex2a(v[7:4]);
                3'd2: nib = hex2a(v[11:8]);
                default: nib = hex2a(v[15:12]);
            endcase
        end
    endfunction

    function [7:0] ch;
        input [5:0] i;
        begin
            if      (i == 6'd0)  ch = 8'h44;                       // 'D'
            else if (i == 6'd1)  ch = present_d ? 8'h31 : 8'h30;   // '1'/'0'
            else if (i == 6'd2)  ch = 8'h2C;                       // ','
            else if (i == 6'd3)  ch = 8'h53;                       // 'S'
            else if (i <= 6'd7)  ch = nib(s_d, 7 - i[2:0]);        // 划痕 4 位 hex
            else if (i == 6'd8)  ch = 8'h2C;                       // ','
            else if (i == 6'd9)  ch = 8'h50;                       // 'P'
            else if (i <= 6'd13) ch = nib(p_d, 13 - i[2:0]);       // 凹坑
            else if (i == 6'd14) ch = 8'h2C;                       // ','
            else if (i == 6'd15) ch = 8'h43;                       // 'C'
            else if (i <= 6'd19) ch = nib(c_d, 19 - i[2:0]);       // 缺角
            else if (i == 6'd20) ch = 8'h0D;                       // '\r'
            else                 ch = 8'h0A;                       // '\n'
        end
    endfunction

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            st <= 2'd0; idx <= 6'd0; uart_send <= 1'b0; uart_data <= 8'd0;
            present_d <= 1'b0; s_d <= 16'd0; p_d <= 16'd0; c_d <= 16'd0;
        end else begin
            case (st)
                2'd0: begin
                    uart_send <= 1'b0;
                    if (rp_tick && !uart_busy) begin
                        present_d <= I_present;
                        s_d <= I_cnt_scratch; p_d <= I_cnt_pit; c_d <= I_cnt_corner;
                        idx <= 6'd0;
                        st  <= 2'd1;
                    end
                end
                2'd1: begin                        // 发出 1 拍 send + 当前字节
                    uart_data <= ch(idx);
                    uart_send <= 1'b1;
                    st <= 2'd2;
                end
                2'd2: begin
                    uart_send <= 1'b0;
                    if (uart_busy) st <= 2'd3;     // 等待 UART 进入忙
                end
                2'd3: begin
                    if (!uart_busy) begin          // 本字节发完
                        if (idx == LEN-1) st <= 2'd0;
                        else begin idx <= idx + 6'd1; st <= 2'd1; end
                    end
                end
            endcase
        end
    end

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart (
        .I_clk (I_clk),
        .I_rst_n(I_rst_n),
        .I_data(uart_data),
        .I_send(uart_send),
        .O_txd (O_uart_txd),
        .O_busy(uart_busy)
    );

endmodule

