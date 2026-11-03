// =============================================================================
// osd_overlay.v  --  OSD 信息叠加（安路 Logo 横幅 + 状态文字 + 目标框 + 帧计数）
// 纯图形层：输入坐标与状态，输出 "该像素是否叠加" 及叠加颜色。
// 上层（hdmi_mixer）据此选择 OSD 颜色或原始图像。
// =============================================================================
module osd_overlay (
    input  wire [11:0] I_x,
    input  wire [11:0] I_y,
    input  wire        I_osd_en,
    input  wire [2:0]  I_mode,
    input  wire [15:0] I_frame,        // 帧计数
    input  wire [15:0] I_count,        // 目标像素个数
    input  wire [2:0]  I_target_sel,   // 目标颜色选择
    input  wire [11:0] I_bx_min,
    input  wire [11:0] I_bx_max,
    input  wire [11:0] I_by_min,
    input  wire [11:0] I_by_max,
    input  wire        I_mv_present,   // 上一帧存在运动块（模式6 MOTION）
    input  wire [15:0] I_mv_cnt,       // 上一帧运动块数（80x45 网格，最大 3600）
    input  wire [3:0]  I_mv_d3,        // 运动块数（十进制）：千位
    input  wire [3:0]  I_mv_d2,        //                     百位
    input  wire [3:0]  I_mv_d1,        //                     十位
    input  wire [3:0]  I_mv_d0,        //                     个位
    // ---- 2026-10-28 新增：目标框坐标（十进制已拆位）与处理延迟 ----
    input  wire [3:0]  I_xd3,          // 框左上角 X：千位
    input  wire [3:0]  I_xd2,          //             百位
    input  wire [3:0]  I_xd1,          //             十位
    input  wire [3:0]  I_xd0,          //             个位
    input  wire [3:0]  I_yd2,          // 框左上角 Y：百位
    input  wire [3:0]  I_yd1,          //             十位
    input  wire [3:0]  I_yd0,          //             个位
    input  wire [3:0]  I_dl2,          // 延迟（0.1ms 单位）：百位
    input  wire [3:0]  I_dl1,          //                    十位
    input  wire [3:0]  I_dl0,          //                    个位
    // ---- 2026-10-31 新增：中央 ROI 平均 RGB（原始像素，模式5 判色调参用） ----
    input  wire [7:0]  I_pxm_r,
    input  wire [7:0]  I_pxm_g,
    input  wire [7:0]  I_pxm_b,
    // ---- 2026-10-31 新增：模式6 取数（判据余量，替代靠截图猜阈值） ----
    input  wire [3:0]  I_mxd2,         // 上一帧最大块差：百位
    input  wire [3:0]  I_mxd1,         //                 十位
    input  wire [3:0]  I_mxd0,         //                 个位
    input  wire [15:0] I_raw,          // 上一帧超阈值块数（未做时间/空间滤波）
    output reg         O_on,           // 是否叠加（文字/框线，实色）
    output reg [23:0]  O_color,        // 叠加颜色
    output reg         O_pan           // 是否落在 OSD 底板上（由 hdmi_mixer 压暗混合）
);

    // ---- 颜色常量 ----
    localparam [23:0] C_WHITE = 24'hFFFFFF;
    localparam [23:0] C_RED   = 24'hE2231D;   // 安路红
    localparam [23:0] C_LIME  = 24'h00FF00;

    // ---- 16bit -> 4 位 hex ASCII（每字符 7bit） ----
    function [6:0] hex2a;
        input [3:0] n;
        begin
            case (n)
                4'h0:hex2a=7'h30; 4'h1:hex2a=7'h31; 4'h2:hex2a=7'h32; 4'h3:hex2a=7'h33;
                4'h4:hex2a=7'h34; 4'h5:hex2a=7'h35; 4'h6:hex2a=7'h36; 4'h7:hex2a=7'h37;
                4'h8:hex2a=7'h38; 4'h9:hex2a=7'h39; 4'hA:hex2a=7'h41; 4'hB:hex2a=7'h42;
                4'hC:hex2a=7'h43; 4'hD:hex2a=7'h44; 4'hE:hex2a=7'h45; 4'hF:hex2a=7'h46;
                default:hex2a=7'h30;
            endcase
        end
    endfunction

    wire [27:0] frm_hex = {hex2a(I_frame[15:12]),hex2a(I_frame[11:8]),
                           hex2a(I_frame[7:4]) ,hex2a(I_frame[3:0])};
    wire [27:0] cnt_hex = {hex2a(I_count[15:12]),hex2a(I_count[11:8]),
                           hex2a(I_count[7:4]) ,hex2a(I_count[3:0])};
    // 注：原来运动块数用 def_hex 显示十六进制，v6 改成十进制（见下面的 mv_word），
    //     def_hex 已删除，省 4 组 hex2a 的 LUT。
    // "C=" + 4 位十六进制（FIND 模式目标像素个数，保留 hex 以省逻辑）
    // 状态字："BUSY"(画面在动) / "IDLE"(画面静止)
    wire [13:0] st_word    = I_mv_present ? {7'h42,7'h55,7'h53,7'h59}
                                          : {7'h49,7'h44,7'h4C,7'h45};

    // ---- 模式 / 目标 名称（运行期选择） ----
    reg [27:0] mode_word;
    reg [27:0] tgt_word;
    always @(*) begin
        case (I_mode)
            3'd0: mode_word = {7'h52,7'h41,7'h57,7'h20};          // "RAW"
            3'd1: mode_word = {7'h47,7'h52,7'h41,7'h59};          // "GRAY"
            3'd2: mode_word = {7'h42,7'h49,7'h4E,7'h20};          // "BIN"
            3'd3: mode_word = {7'h45,7'h44,7'h47,7'h45};          // "EDGE"
            3'd4: mode_word = {7'h54,7'h4F,7'h4F,7'h4E};          // "TOON"
            3'd5: mode_word = {7'h46,7'h49,7'h4E,7'h44};          // "FIND"
            3'd6: mode_word = {7'h4D,7'h4F,7'h54,7'h4E};          // "MOTN"（运动检测）
            default: mode_word = {7'h52,7'h41,7'h57,7'h20};
        endcase
        case (I_target_sel)
            3'd0: tgt_word = {7'h52,7'h45,7'h44,7'h20};           // "RED"
            3'd1: tgt_word = {7'h47,7'h52,7'h4E,7'h20};           // "GRN"
            3'd2: tgt_word = {7'h42,7'h4C,7'h55,7'h20};           // "BLU"
            3'd3: tgt_word = {7'h59,7'h45,7'h4C,7'h20};           // "YEL"
            // 3'd4 CYN 已按需求删除（KEY4 不再能选到青色，故不显示该标签）
            3'd5: tgt_word = {7'h57,7'h48,7'h54,7'h20};           // "WHT"
            default: tgt_word = {7'h52,7'h45,7'h44,7'h20};
        endcase
    end

    // ---- 是否已锁定目标（用于 OBJ 显示切换 颜色/----，仅 FIND 模式有意义） ----
    wire obj_found = (I_mode == 3'd5) && (I_bx_max > I_bx_min) && (I_by_max > I_by_min);

    // =========================================================================
    // 2026-10-28 新增：目标框坐标行 "XY 620,310" 与处理延迟行 "DLY 16.6MS"
    //   坐标与延迟的十进制拆位在 hdmi_mixer 里由 bin2bcd（12 拍串行）算好，
    //   这里只做 BCD -> ASCII 与前导零变空格，纯组合、不引入额外时序压力。
    // =========================================================================
    wire [6:0] C_SP = 7'h20;
    // X：千位/百位前导零显示为空格，十位/个位恒显示
    wire [6:0] x3a = (I_xd3 != 4'd0)                        ? ({3'd0,I_xd3} + 7'h30) : C_SP;
    wire [6:0] x2a = ((I_xd3 != 4'd0) || (I_xd2 != 4'd0))   ? ({3'd0,I_xd2} + 7'h30) : C_SP;
    wire [6:0] x1a = {3'd0,I_xd1} + 7'h30;
    wire [6:0] x0a = {3'd0,I_xd0} + 7'h30;
    // Y：百位前导零变空格
    wire [6:0] y2a = (I_yd2 != 4'd0)                        ? ({3'd0,I_yd2} + 7'h30) : C_SP;
    wire [6:0] y1a = {3'd0,I_yd1} + 7'h30;
    wire [6:0] y0a = {3'd0,I_yd0} + 7'h30;
    // 延迟：百位前导零变空格
    wire [6:0] d2a = (I_dl2 != 4'd0)                        ? ({3'd0,I_dl2} + 7'h30) : C_SP;
    wire [6:0] d1a = {3'd0,I_dl1} + 7'h30;
    wire [6:0] d0a = {3'd0,I_dl0} + 7'h30;

    // "XY " + X(4) + "," + Y(3) = 11 字符
    // 未锁定目标时整行置空格（color_obj_detect 无目标会把 bbox 清 0，
    // 直接显示会出现 "XY 0,0"，容易误读成"目标在左上角"）
    wire [76:0] xy_word_raw = {7'h58, 7'h59, C_SP, x3a, x2a, x1a, 7'h2C, y2a, y1a, y0a};
    wire [76:0] xy_word     = obj_found ? xy_word_raw : {11{C_SP}};
    // "DLY " + d.d + "MS" = 10 字符
    wire [69:0] dly_word = {7'h44, 7'h4C, 7'h59, C_SP, d2a, d1a, 7'h2E, d0a, 7'h4D, 7'h53};

    // ---- 模式6 运动块数：十进制 4 位（0~3600），前导零显示为空格 ----
    //   改成十进制的原因：原来显示十六进制（N=04EB），现场没法一眼判断
    //   "1259 块到底是多还是少"，沟通成本太高。BCD 由 hdmi_mixer 的 bin2bcd 算好。
    wire [6:0] m3a = (I_mv_d3 != 4'd0)                              ? ({3'd0,I_mv_d3} + 7'h30) : C_SP;
    wire [6:0] m2a = ((I_mv_d3 != 4'd0) || (I_mv_d2 != 4'd0))       ? ({3'd0,I_mv_d2} + 7'h30) : C_SP;
    wire [6:0] m1a = {3'd0,I_mv_d1} + 7'h30;
    wire [6:0] m0a = {3'd0,I_mv_d0} + 7'h30;
    // "MV=" + 4 位十进制 = 7 字符
    wire [48:0] mv_word = {7'h4D, 7'h56, 7'h3D, m3a, m2a, m1a, m0a};

    // ---- 各文字实例 ----
    wire banner_on, mode_lbl_on, mode_val_on, fps_lbl_on, fps_val_on;
    wire f_lbl_on, f_val_on, obj_lbl_on, obj_tgt_on, n_lbl_on, n_val_on;
    wire def_n_on, def_st_on;   // 模式6 缺陷计数 / OK-NG 文本
    wire xy_on, dly_on;         // 2026-10-28 新增：坐标行 / 延迟行
    wire ver_on;                // 2026-10-30 新增：固件版本标识

    osd_text_rt #(7) u_banner (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd10), .I_y0(12'd10),
        .I_chars({7'h41,7'h4E,7'h4C,7'h4F,7'h47,7'h49,7'h43}), .O_on(banner_on)); // "ANLOGIC"

    osd_text_rt #(5) u_modeL (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd10), .I_y0(12'd26),
        .I_chars({7'h4D,7'h4F,7'h44,7'h45,7'h3A}), .O_on(mode_lbl_on));           // "MODE:"

    osd_text_rt #(4) u_modeV (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd54), .I_y0(12'd26),
        .I_chars(mode_word), .O_on(mode_val_on));

    osd_text_rt #(4) u_fpsL (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd10), .I_y0(12'd38),
        .I_chars({7'h46,7'h50,7'h53,7'h3A}), .O_on(fps_lbl_on));                   // "FPS:"

    osd_text_rt #(2) u_fpsV (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd46), .I_y0(12'd38),
        .I_chars({7'h36,7'h30}), .O_on(fps_val_on));                                // "60"

    osd_text_rt #(2) u_fL (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd10), .I_y0(12'd50),
        .I_chars({7'h46,7'h3A}), .O_on(f_lbl_on));                                  // "F:"

    osd_text_rt #(4) u_fV (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd26), .I_y0(12'd50),
        .I_chars(frm_hex), .O_on(f_val_on));

    osd_text_rt #(4) u_objL (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd10), .I_y0(12'd62),
        .I_chars({7'h4F,7'h42,7'h4A,7'h3A}), .O_on(obj_lbl_on));                   // "OBJ:"

    osd_text_rt #(4) u_objT (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd46), .I_y0(12'd62),
        .I_chars(tgt_word), .O_on(obj_tgt_on));   // 跟随 KEY4 选择（RED/GRN/BLU/YEL/WHT，已去掉 CYN）

    osd_text_rt #(2) u_nL (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd90), .I_y0(12'd62),
        .I_chars({7'h43,7'h3D}), .O_on(n_lbl_on));                                  // "C="（原 "N="，避免与下面的运动块数混淆）

    osd_text_rt #(4) u_nV (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd106), .I_y0(12'd62),
        .I_chars(cnt_hex), .O_on(n_val_on));

    // ---- 2026-10-28 新增：目标框坐标行（仅 FIND 模式）与处理延迟行（常显） ----
    osd_text_rt #(11) u_xy (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd10), .I_y0(12'd86),
        .I_chars(xy_word), .O_on(xy_on));

    osd_text_rt #(10) u_dly (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd10), .I_y0(12'd98),
        .I_chars(dly_word), .O_on(dly_on));

    // ---- 模式6：运动块数 "MV=dddd" + 状态 "IDLE"/"BUSY" ----
    osd_text_rt #(7) u_dN (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd10), .I_y0(12'd74),
        .I_chars(mv_word), .O_on(def_n_on));
    osd_text_rt #(4) u_dSt (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd92), .I_y0(12'd74),
        .I_chars(st_word), .O_on(def_st_on));

    // ---- 固件版本标识（放在 ANLOGIC 横幅右边，红底白字） ----
    //   存在的唯一目的：上板后一眼确认烧进去的是哪一版，避免"改了没生效"
    //   和"没烧对"两种问题无法区分（v4/v5 两轮就是卡在这里）。
    osd_text_rt #(3) u_ver (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd74), .I_y0(12'd10),
        .I_chars({7'h56,7'h31,7'h33}), .O_on(ver_on));     // "V13"

    // ---- 2026-10-31 新增：模式5 判色"取数"行 ----
    //   判色阈值一直是靠猜的（蓝认不出、白被认成黄，两轮都是盲调）。
    //   这行把"上一帧中央 ROI 的平均原始 RGB"直接打到屏上：
    //     · 镜头对准蓝屏/白纸/黄卡，读 M:RRGGBB，就能算出 sat/max、B-R、B-G；
    //     · 阈值据此定，不用再靠截图反推（显示链有 gamma/饱和度/白平衡，
    //       从截图反推原始像素必然不准）。
    //   只在 FIND 模式显示，其它模式不占屏。
    wire [55:0] mrg_word = {7'h4D, 7'h3A,
                            hex2a(I_pxm_r[7:4]), hex2a(I_pxm_r[3:0]),
                            hex2a(I_pxm_g[7:4]), hex2a(I_pxm_g[3:0]),
                            hex2a(I_pxm_b[7:4]), hex2a(I_pxm_b[3:0])};
    wire mrg_on;
    osd_text_rt #(8) u_mrg (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd10), .I_y0(12'd110),
        .I_chars(mrg_word), .O_on(mrg_on));       // "M:" + RRGGBB

    // ---- 2026-10-31 新增：模式6 取数行 "D=nnn R=xxxx" ----
    //   D = 上一帧出现过的**最大块差**（十进制 LSB）。静止场景这个数就是"噪声底"：
    //       它若在 15~30 之间，说明阈值 24 已经够；若静止时还有 60+，说明是结构性
    //       变化（曝光/灯闪/对齐），抬阈值没用，得换思路。
    //   R = 上一帧"块差超阈值"的块数（十六进制，未做时间/空间滤波）。
    //       静止时应接近 0（阈值 24 下），跟 MV 一比就知道滤波帮你压掉了多少。
    wire [6:0] q2a = (I_mxd2 != 4'd0) ? ({3'd0,I_mxd2} + 7'h30) : C_SP;
    wire [6:0] q1a = ((I_mxd2 != 4'd0) || (I_mxd1 != 4'd0)) ? ({3'd0,I_mxd1} + 7'h30) : C_SP;
    wire [6:0] q0a = {3'd0,I_mxd0} + 7'h30;
    wire [34:0] d_word = {7'h44, 7'h3D, q2a, q1a, q0a};              // "D=nnn"
    wire [41:0] r_word = {7'h52, 7'h3D,
                          hex2a(I_raw[15:12]), hex2a(I_raw[11:8]),
                          hex2a(I_raw[7:4]),   hex2a(I_raw[3:0])};   // "R=xxxx"
    wire d_on, r_on;
    osd_text_rt #(5) u_dbg_d (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd10), .I_y0(12'd86),
        .I_chars(d_word), .O_on(d_on));
    osd_text_rt #(6) u_dbg_r (
        .I_x(I_x), .I_y(I_y), .I_x0(12'd58), .I_y0(12'd86),
        .I_chars(r_word), .O_on(r_on));

    // 模式6 判定 / 运动提示边框（画面在动时整屏红色细边框）
    wire in_defmode   = (I_mode == 3'd6);
    wire in_find      = (I_mode == 3'd5);   // 2026-10-28：坐标行仅 FIND 模式显示
    wire alarm_border = in_defmode && I_mv_present &&
                        (I_x < 12'd6 || I_x > 12'd1273 || I_y < 12'd6 || I_y > 12'd713);

    // ---- Logo 红底横幅（图形） ----
    //   2026-10-30：右边界从 78 扩到 96，把版本标识 "V6" 也包进红底
    wire banner_bg = (I_x >= 12'd8 && I_x <= 12'd104 && I_y >= 12'd8 && I_y <= 12'd19);

    // ---- 目标检测框（仅 FIND 模式且已锁定目标时绘制） ----
    wire box_h  = (I_y == I_by_min || I_y == I_by_max) &&
                  (I_x >= I_bx_min && I_x <= I_bx_max);
    wire box_v  = (I_x == I_bx_min || I_x == I_bx_max) &&
                  (I_y >= I_by_min && I_y <= I_by_max);
    wire box_on = obj_found && (box_h || box_v);

    // ---- 汇总叠加 ----
    always @(*) begin
        O_on = 1'b0;
        O_color = 24'd0;
        O_pan = 1'b0;
        if (I_osd_en) begin
            // 2026-11-02：OSD 底板范围，覆盖全部文字行（x 4~146 / y 4~124）。
            //   模式 2/4 的画面接近纯白时，白字压在白底上整行消失，
            //   评委在画面上根本看不到当前模式名。压暗混合放在 hdmi_mixer
            //   里做（osd_pan 与 O_on 同拍），叠加层本身仍是纯白字。
            O_pan = (I_x >= 12'd4 && I_x <= 12'd146 &&
                     I_y >= 12'd4 && I_y <= 12'd124);
            if (banner_bg) begin
                O_on = 1'b1; O_color = C_RED;     // 红底
            end
            // 文字（白）优先级高于红底
            // 2026-11-02：OBJ 类别与 C= 计数只有 FIND 模式有意义，其它模式不再显示
            if (banner_on  | mode_lbl_on | mode_val_on | fps_lbl_on | fps_val_on |
                f_lbl_on   | f_val_on    | ver_on     | (mrg_on && in_find) |
                ((obj_lbl_on | obj_tgt_on | n_lbl_on | n_val_on) && in_find) |
                (d_on && in_defmode) | (r_on && in_defmode)) begin
                O_on = 1'b1; O_color = C_WHITE;
            end
            // 模式6：运动块数（白）与 IDLE/BUSY 状态（静止绿、运动中红）
            if (in_defmode && def_n_on) begin
                O_on = 1'b1; O_color = C_WHITE;
            end
            if (in_defmode && def_st_on) begin
                O_on = 1'b1; O_color = I_mv_present ? C_RED : 24'h00FF00;
            end
            if (box_on) begin
                O_on = 1'b1; O_color = C_LIME;    // 检测框
            end
            // 2026-10-28：坐标行 / 延迟行（压在框线之上，保证数字不被切断）
            if (in_find && xy_on) begin O_on = 1'b1; O_color = C_WHITE;   end
            if (dly_on)           begin O_on = 1'b1; O_color = 24'h00FFFF; end
            // 报警红边框（最高优先级）
            if (alarm_border) begin
                O_on = 1'b1; O_color = C_RED;
            end
        end
    end

endmodule
