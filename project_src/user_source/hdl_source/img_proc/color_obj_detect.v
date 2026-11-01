// =============================================================================
// color_obj_detect.v  --  彩色目标检测 + 自动颜色识别（标准 HSV 色相，光照无关）
//
// 改进点（v3）：
//   1) 用标准 HSV 色相公式 h 做 6 色分类，以 30° 为界划入 R/Y/G/C/B/M：
//        max=R: h = 60(G-B)/d            (负值 +360)
//        max=G: h = 60(B-R)/d + 120
//        max=B: h = 60(R-G)/d + 240
//      （d = max-min）全部化为"2*(mid-min) 与 d 比大小"，只用减法/移位/比较，
//      不占 DSP、不除法，纯组合。
//      >> 旧版( ) 的致命 bug：用 max 通道次序直接判色，红(200,30,30) 因 G>=B 被
//         判成"黄"，绿(30,200,30) 因 B>=R 被判成"青"——红/绿永远认不出。
//   2) 归一化纯度：sat/max >= 1/8 才算"有彩色"，与亮度、曝光完全无关。
//      v4(2026-10-28)：判色源改为传感器原始像素（不经白平衡偏色增益/AEC增益），
//      白/灰改为"纯度<1/6 且 max>=110"，避免阴影和深色物体被当成白色。
//   3) 最小面积过滤 MIN_AREA：单点/碎点噪声不画框，框稳定不抖。
//   4) 区域均值识别：累加命中像素 RGB，帧首用同一套色相逻辑判定物体真实颜色，
//      经 O_recog 输出给 OSD。
//   5) v10(2026-10-31)：蓝改为"只要求 B 明显高于 R"，不再要求 B 比 G 高 26
//      （见下方 blu_ok 注释）。
//   6) v11(2026-10-31)：黄加"纯度"门槛 sat/max>=0.40，把暖光下的白纸/白墙
//      挡在黄色之外（见下方 yel_ok 注释）。
//
// 颜色编码（与 I_target_sel 一致）： 0 RED 1 GRN 2 BLU 3 YEL 4 CYN 5 WHT
// =============================================================================
module color_obj_detect (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_pix_en,        // 有效像素使能
    input  wire        I_user,          // 帧起始（首个有效像素）
    input  wire [11:0] I_x,
    input  wire [11:0] I_y,
    input  wire [23:0] I_rgb,
    input  wire [2:0]  I_target_sel,
    output reg  [11:0] O_bx_min,
    output reg  [11:0] O_bx_max,
    output reg  [11:0] O_by_min,
    output reg  [11:0] O_by_max,
    output reg  [15:0] O_count,
    output reg  [2:0]  O_recog,         // 自动识别出的颜色（与编码一致）
    output wire        O_match
);

    // -------------------------------------------------------------------------
    // 可调阈值
    // -------------------------------------------------------------------------
    localparam [7:0]  MIN_VAL     = 8'd46;   // 太暗的像素不算目标（噪声/阴影）
    localparam [8:0]  MIN_SAT     = 9'd20;   // 绝对纯度下限（max-min）：仅抑制暗部噪声
    localparam [15:0] MIN_AREA    = 16'd100; // 最小有效面积（像素），滤除噪声/碎点
    // 只在"画面中央 ROI"内统计目标，避免背景角落里零星误匹配把包围盒撑到全屏
    localparam [11:0] ROI_X0 = 12'd128, ROI_X1 = 12'd1151;   // 中央约 80% 宽
    localparam [11:0] ROI_Y0 = 12'd72,  ROI_Y1 = 12'd647;    // 中央约 80% 高
    // 横向连续段过滤：连续匹配像素达到 RUN_MIN 才算"有效"，孤立点/短线不计入框
    localparam [7:0]  RUN_MIN = 8'd5;
    // ---- 宽容版颜色判据（"给每种颜色定个正常标准，接近它的也算"）----
    //   · 彩色判据用「比例 sat/max >= 1/6」：比例不随 AEC 数字增益缩放，亮场景里
    //     增益被压低时不会把彩色误杀（这是"只有白色认得出"的关键根因之一）。
    //     另加很小的绝对下限 MIN_SAT 抑制暗部噪声。
    //   · 归类用「主通道优势」：某通道比另两通道各高出 >= MARGIN 即算该色，
    //     因此偏橙/偏粉都算红、偏黄的绿都算绿——"接近那个颜色的也算"。
    //   · 黄：R、G 都明显高于 B，且两者接近（|R-G| <= YEL_TOL）。
    //   · 白/灰：不满足彩色判据（sat/max < 1/6）。
    //   MARGIN/YEL_TOL 越小越宽容；调大可更挑剔。
    localparam [8:0]  MARGIN      = 9'd26;   // 主通道需比另两通道各高出至少这么多
    localparam [8:0]  YEL_TOL     = 9'd52;   // 黄：|R-G| 不超过该值（且 R,G 都高于 B）
    localparam [8:0]  GB_TOL      = 9'd45;   // 红：|G-B| 不超过该值（排除橙色，橙的 G>>B）
    localparam [8:0]  BLU_RM      = 9'd40;   // 蓝：B 至少比 R 高这么多（替掉旧的 B-G 判据）

    reg [11:0] cur_minx, cur_maxx, cur_miny, cur_maxy;
    reg [15:0] cur_cnt;
    reg [31:0] cur_sumr, cur_sumg, cur_sumb; // 32bit：满屏 x255 也不溢出
    reg        user_d;

    wire [7:0] R = I_rgb[23:16];
    wire [7:0] G = I_rgb[15:8];
    wire [7:0] B = I_rgb[7:0];

    // -------------------------------------------------------------------------
    // 通用色相分类（纯组合、无除法）
    //   输入三通道"同量纲"值（像素 0..255，或区域累加和 >>5 后的值）
    //   输出：0 RED / 1 GRN / 2 BLU / 3 YEL / 4 CYN  （白/灰由调用处按纯度判 5）
    // -------------------------------------------------------------------------
    function [2:0] hue_cls;
        input [23:0] r;
        input [23:0] g;
        input [23:0] b;
        reg  [23:0] mx, mn, d, m1;
        reg  [25:0] t2;      // 2*(中间通道 - 最小通道)，比 d 用
        begin
            mx = (r>=g && r>=b) ? r : (g>=r && g>=b) ? g : b;
            mn = (r<=g && r<=b) ? r : (g<=r && g<=b) ? g : b;
            d  = mx - mn;
            t2 = {2'd0, d};
            if (mx == r) begin
                if (g >= b) begin
                    m1 = g - b;
                    t2 = {2'd0, m1} << 1;                       // 2*(G-B)
                    hue_cls = (t2 < {2'd0, d}) ? 3'd0 : 3'd3;   // h<30 红，否则黄
                end else begin
                    m1 = b - g;
                    t2 = {2'd0, m1} << 1;                       // 2*(B-G)
                    hue_cls = (t2 < {2'd0, d}) ? 3'd0 : 3'd2;   // 近红归红，品红归蓝
                end
            end else if (mx == g) begin
                if (r >= b) begin
                    m1 = r - b;
                    t2 = {2'd0, m1} << 1;                       // 2*(R-B)
                    // h = 120 - 60(R-B)/d ：h<90 才是黄
                    hue_cls = (t2 > {2'd0, d}) ? 3'd3 : 3'd1;
                end else begin
                    m1 = b - r;
                    t2 = {2'd0, m1} << 1;                       // 2*(B-R)
                    // h = 120 + 60(B-R)/d ：h>150 转青
                    hue_cls = (t2 < {2'd0, d}) ? 3'd1 : 3'd4;
                end
            end else begin
                if (g >= r) begin
                    m1 = g - r;
                    t2 = {2'd0, m1} << 1;                       // 2*(G-R)
                    // h = 240 - 60(G-R)/d ：h<210 是青
                    hue_cls = (t2 > {2'd0, d}) ? 3'd4 : 3'd2;
                end else begin
                    m1 = r - g;
                    t2 = {2'd0, m1} << 1;                       // 2*(R-G)
                    // h = 240 + 60(R-G)/d ：h>270 归红（品红）
                    hue_cls = (t2 < {2'd0, d}) ? 3'd2 : 3'd0;
                end
            end
        end
    endfunction

    // ---- 像素级判定 ----
    wire [7:0] px_mx = (R>=G && R>=B) ? R : (G>=R && G>=B) ? G : B;
    wire [7:0] px_mn = (R<=G && R<=B) ? R : (G<=R && G<=B) ? G : B;
    wire [8:0] px_sat = px_mx - px_mn;
    wire       px_dark   = (px_mx < MIN_VAL);
    // 彩色判据（与增益、曝光无关）：纯度 sat/max >= 1/8（12.5%）。
    //   比例不随增益缩放，所以"亮场景增益被压低"时彩色不会被误杀。
    //   另加很小的绝对下限 MIN_SAT 抑制暗部噪声。
    //   ★ 2026-10-28：判色源改成传感器原始像素（不再经过白平衡偏色增益与 AEC 增益），
    //     因此这里看到的就是物体本身的纯度，门槛才能定死在 1/8。
    wire [12:0] sat8 = {4'd0, px_sat} * 13'd8;   // sat*8
    wire [12:0] sat4 = {4'd0, px_sat} * 13'd4;   // sat*4（白/灰判据，v11）
    wire [12:0] mx13 = {5'd0, px_mx};            // max（加宽到 13bit 比较）
    wire       px_chroma = (!px_dark) && (px_sat >= MIN_SAT) && (sat8 >= mx13);
    // 白/灰：纯度 sat/max < 1/4，且不能太暗。
    //   ★ 旧版是"只要不彩色就算白"，于是昏暗的桌沿、黑胶带、阴影全被算成白色，
    //     选 WHT 时包围盒会拉到整屏。加 max>=110 后只认真正明亮的白/灰面。
    //   ★ v11：门槛从 1/6 放宽到 1/4。原因：灯管/暖光照在白纸上，原始像素
    //     本身就带 20%~25% 的偏色（典型 (220,210,170)，sat/max=0.23），
    //     1/6 的门槛会让"白纸"既不算白(>1/6)、也才是黄 —— 选 WHT 时反而没反应。
    //     放宽到 1/4 后白纸归白，黄色统一交给 yel_ok 的 0.40 门槛，
    //     两者之间留 0.25~0.40 的"模糊带"（既不框白也不框黄），互不抢。
    wire       px_white  = (!px_dark) && (sat4 < mx13) && (px_mx >= 8'd110);

    // 逐通道 9bit 加宽：R9>=G9+MARGIN 若在 8bit 里算会溢出回绕
    wire [8:0] R9 = {1'b0, R};
    wire [8:0] G9 = {1'b0, G};
    wire [8:0] B9 = {1'b0, B};
    wire [8:0] dRG = (R >= G) ? (R - G) : (G - R);
    wire [8:0] dGB = (G >= B) ? (G - B) : (B - G);

    // 各标准色的“宽容”判据：主通道领先另两通道 >= MARGIN
    // 红额外加两条，专门排除“橙”和“粉”（按需求：只认真正的红）：
    //   · |G-B| <= GB_TOL  —— 橙色 G 明显高于 B（G-B 很大），会被排除；
    //   · B*10 <= R*7（B <= 0.7R）—— 粉色是“亮红且蓝偏高”（B≈R），会被排除；
    //     真正的红 B 很低（B/R≈0.2），深红/暗红也能通过。
    wire red_ok = (R9 >= G9 + MARGIN) && (R9 >= B9 + MARGIN) &&
                  (dGB <= GB_TOL) &&
                  (({4'd0,B} * 12'd10) <= ({4'd0,R} * 12'd7));
    wire grn_ok = (G9 >= R9 + MARGIN) && (G9 >= B9 + MARGIN);
    // 蓝：v10 修正（"手机上的蓝底图片识别不出来"的根因）
    //   旧判据 = B >= R+MARGIN 且 B >= G+MARGIN，即要求**蓝比绿高 26**。
    //   但相机对绿光最敏感、蓝通道量子效率最低，加上现场是日光灯（光谱偏绿），
    //   真实蓝色物体的**原始像素**里 B 往往只比 G 高个位数，甚至被 G 反超。
    //   白平衡是在这之后才把蓝拉起来的 —— 人眼看屏幕是蓝的，而判色拿到的是
    //   "接近青灰"的一坨数，B>=G+26 永远不成立，于是整片蓝色一个像素都不命中。
    //   近距离怼屏幕时更糟：B 和 G 一起削顶到 255，差值直接是 0。
    //   新判据只要求"蓝明显高于红"（>=BLU_RM），允许 G 比 B 高出 MARGIN 以内：
    //     · #0264C4 这类偏青的蓝、深蓝、天蓝、淡蓝都能命中
    //     · 绿(G>B)、黄、红、白、灰仍被排除（它们 B 都不比 R 高 40 以上）
    wire blu_ok = (B9 >= R9 + BLU_RM) && (B9 + MARGIN >= G9);
    // 黄：v11 修正（"选黄色时白纸/白墙也被框住"的根因）
    //   旧判据只用**绝对差**（B 比 R、G 各低 MARGIN=26 就算黄）。
    //   但灯管/暖光照在白纸上，传感器原始像素本来就是 R>G>B，
    //   典型 (220,210,170)：B 比 R 低 50、比 G 低 40、|R-G|=10 —— 26 的门槛
    //   完全不设防，白纸 100% 落进 yel_ok，"白色被认成黄色"就是这么来的。
    //   区分二者的不是色相（暖白 ~45°、黄 ~50°，几乎重合）而是**纯度**：
    //   白纸的 sat/max 只有 20%~30%，真正的黄色物体（屏幕黄、彩色卡纸、
    //   黄便签）都在 50% 以上。
    //   因此补一条**比例**门槛：蓝通道亏损必须占 max 的 40% 以上。
    //   比例与曝光/增益无关（增益同时缩放三个通道，比值不变），
    //   白纸(≤30%)被挡在门外，黄(≥45%)照旧命中。
    //   ★ 调档：黄不够灵 -> 0.40 降到 0.36；白还是被框 -> 升到 0.44。
    wire [12:0] sat5    = {3'd0, px_sat} * 13'd5;    // sat*5（最大 255*5=1275）
    wire [12:0] mx_x2   = {2'd0, px_mx, 1'b0};       // max*2（最大 510）
    wire        yel_pur = (sat5 >= mx_x2);           // sat/max >= 0.40
    wire yel_ok = (R9 >= B9 + MARGIN) && (G9 >= B9 + MARGIN) &&
                  (dRG <= YEL_TOL) && yel_pur && !red_ok && !grn_ok;

    // ---- 目标匹配（白走低饱和通路；其余按各自标准宽容匹配，接近色也算）----
    wire match = (!px_dark) &&
        ( (I_target_sel == 3'd5) ? px_white :
          (I_target_sel == 3'd0) ? (px_chroma && red_ok) :
          (I_target_sel == 3'd1) ? (px_chroma && grn_ok) :
          (I_target_sel == 3'd2) ? (px_chroma && blu_ok) :
          (I_target_sel == 3'd3) ? (px_chroma && yel_ok) :
          (I_target_sel == 3'd4) ? (px_chroma && (grn_ok || blu_ok)) : // 青并入绿/蓝（已不可选）
                                   1'b0 );
    assign O_match = match;

    // ---- ROI 限定 + 横向连续段过滤（去孤立点，收紧包围盒）----
    wire in_roi = (I_x >= ROI_X0) && (I_x <= ROI_X1) &&
                  (I_y >= ROI_Y0) && (I_y <= ROI_Y1);
    wire m = match && in_roi;

    reg [7:0] run;      // 当前行连续匹配像素数（行消隐时清零）
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)         run <= 8'd0;
        else if (!I_pix_en)   run <= 8'd0;                    // 每行/消隐清零
        else if (m)           run <= (run == 8'hFF) ? 8'hFF : run + 8'd1;
        else                  run <= 8'd0;
    end
    // 当前像素使连续段达到 RUN_MIN，才算"有效目标像素"
    wire solid = m && (run >= RUN_MIN - 8'd1);

    // ---- 区域累加和 -> 24bit 同量纲（>>5），供识别复用同一套色相逻辑 ----
    wire [23:0] rs = cur_sumr[28:5];
    wire [23:0] gs = cur_sumg[28:5];
    wire [23:0] bs = cur_sumb[28:5];
    wire [23:0] smx = (rs>=gs && rs>=bs) ? rs : (gs>=rs && gs>=bs) ? gs : bs;
    wire [23:0] smn = (rs<=gs && rs<=bs) ? rs : (gs<=rs && gs<=bs) ? gs : bs;
    // 区域归一化纯度：同样用 sat/max >= 1/4，与目标面积大小无关
    wire [23:0] sdif = smx - smn;
    wire [23:0] sm_th = (smx >> 2);                 // max/4  ->  sat/max >= 1/4
    wire [2:0]  sum_cls = (sdif >= sm_th) ? hue_cls(rs, gs, bs) : 3'd5;

    // ---- 帧首锁存 + 累加 ----
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            user_d   <= 1'b0;
            cur_minx <= 12'hFFF; cur_maxx <= 12'h000;
            cur_miny <= 12'hFFF; cur_maxy <= 12'h000;
            cur_cnt  <= 16'd0;
            cur_sumr <= 32'd0; cur_sumg <= 32'd0; cur_sumb <= 32'd0;
            O_bx_min <= 12'h000; O_bx_max <= 12'h000;
            O_by_min <= 12'h000; O_by_max <= 12'h000;
            O_count  <= 16'd0;
            O_recog  <= 3'd5;
        end else begin
            user_d <= I_user;
            if (I_user && !user_d) begin
                // 仅当达到最小面积才锁存为有效目标，否则清空（滤除噪声/误检）
                if (cur_cnt >= MIN_AREA) begin
                    O_bx_min <= cur_minx; O_bx_max <= cur_maxx;
                    O_by_min <= cur_miny; O_by_max <= cur_maxy;
                    O_count  <= cur_cnt;
                    O_recog  <= sum_cls;
                end else begin
                    O_bx_min <= 12'h000; O_bx_max <= 12'h000;
                    O_by_min <= 12'h000; O_by_max <= 12'h000;
                    O_count  <= 16'd0;
                    O_recog  <= 3'd5;
                end
                // 清空本帧累加器
                cur_minx <= 12'hFFF; cur_maxx <= 12'h000;
                cur_miny <= 12'hFFF; cur_maxy <= 12'h000;
                cur_cnt  <= 16'd0;
                cur_sumr <= 32'd0; cur_sumg <= 32'd0; cur_sumb <= 32'd0;
            end else if (I_pix_en && solid) begin
                cur_cnt  <= cur_cnt + 16'd1;
                cur_sumr <= cur_sumr + {24'd0, R};
                cur_sumg <= cur_sumg + {24'd0, G};
                cur_sumb <= cur_sumb + {24'd0, B};
                if (I_x < cur_minx) cur_minx <= I_x;
                if (I_x > cur_maxx) cur_maxx <= I_x;
                if (I_y < cur_miny) cur_miny <= I_y;
                if (I_y > cur_maxy) cur_maxy <= I_y;
            end
        end
    end

endmodule
