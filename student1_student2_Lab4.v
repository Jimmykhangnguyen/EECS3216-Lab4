module Lab4 (
    input         MAX10_CLK1_50,
    input  [1:0]  KEY,
    input  [9:0]  SW,
    output [9:0]  LEDR,
    output [7:0]  HEX0,
    output [7:0]  HEX1,
    output [7:0]  HEX2,
    output [7:0]  HEX3,
    output [7:0]  HEX4,
    output [7:0]  HEX5
);
 
    //------------------------------------------------------------------
    // Parameters (change CLK_HZ / divide for simulation if desired)
    //------------------------------------------------------------------
    parameter CLK_HZ      = 50_000_000;
    parameter ONE_SEC     = CLK_HZ;             // 1 s
    parameter HALF_SEC    = CLK_HZ / 2;         // 0.5 s
    parameter TENTH_SEC   = CLK_HZ / 10;        // 0.1 s
    parameter ERR_HALF    = CLK_HZ / 5;         // 0.2 s per error-blink half period
    parameter DEBOUNCE    = CLK_HZ / 50;        // 20 ms lock-out after a press
 
    wire clk   = MAX10_CLK1_50;
    wire rst_n = KEY[0];                         // asynchronous, active low
 
    //------------------------------------------------------------------
    // 7-segment codes (active low, bit7 = dp, {dp,g,f,e,d,c,b,a})
    //------------------------------------------------------------------
    localparam SEG_BLANK = 8'hFF;
    localparam SEG_DASH  = 8'hBF;
    localparam SEG_O     = 8'hC0;
    localparam SEG_P     = 8'h8C;
    localparam SEG_F     = 8'h8E;
    localparam SEG_L     = 8'hC7;
    localparam SEG_E     = 8'h86;
    localparam SEG_R     = 8'hAF;
 
    function [7:0] seg_digit;
        input [3:0] d;
        begin
            case (d)
                4'd0: seg_digit = 8'hC0;
                4'd1: seg_digit = 8'hF9;
                4'd2: seg_digit = 8'hA4;
                4'd3: seg_digit = 8'hB0;
                4'd4: seg_digit = 8'h99;
                4'd5: seg_digit = 8'h92;
                4'd6: seg_digit = 8'h82;
                4'd7: seg_digit = 8'hF8;
                4'd8: seg_digit = 8'h80;
                4'd9: seg_digit = 8'h90;
                default: seg_digit = SEG_BLANK;
            endcase
        end
    endfunction
 
    //------------------------------------------------------------------
    // KEY1 : synchronizer + falling-edge detect + debounce lock-out
    //------------------------------------------------------------------
    reg [2:0]  key_sync;
    reg [24:0] lock_cnt;
    reg        coin_pulse;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_sync   <= 3'b111;
            lock_cnt   <= 25'd0;
            coin_pulse <= 1'b0;
        end else begin
            key_sync   <= {key_sync[1:0], KEY[1]};
            coin_pulse <= 1'b0;
            if (lock_cnt != 0)
                lock_cnt <= lock_cnt - 25'd1;
            else if (key_sync[2] & ~key_sync[1]) begin   // falling edge = press
                coin_pulse <= 1'b1;
                lock_cnt   <= DEBOUNCE;
            end
        end
    end
 
    //------------------------------------------------------------------
    // Free-running blink generators (0.5 s and 0.1 s on/off)
    //------------------------------------------------------------------
    reg [25:0] half_cnt, tenth_cnt;
    reg        blink_half, blink_tenth;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            half_cnt <= 0;  blink_half  <= 1'b0;
            tenth_cnt <= 0; blink_tenth <= 1'b0;
        end else begin
            if (half_cnt == HALF_SEC-1) begin
                half_cnt <= 0; blink_half <= ~blink_half;
            end else half_cnt <= half_cnt + 1'b1;
 
            if (tenth_cnt == TENTH_SEC-1) begin
                tenth_cnt <= 0; blink_tenth <= ~blink_tenth;
            end else tenth_cnt <= tenth_cnt + 1'b1;
        end
    end
 
    //------------------------------------------------------------------
    // Inputs decoding
    //------------------------------------------------------------------
    wire [2:0] occ       = SW[2:0];
    wire [1:0] sel       = SW[9:8];
    wire       sel_valid = (sel != 2'b00);
    wire [1:0] sel_idx   = sel - 2'd1;                 // 0,1,2 for sel = 1,2,3
    wire       sel_occ   = sel_valid && occ[sel_idx];
 
    //------------------------------------------------------------------
    // Per-space timers
    //------------------------------------------------------------------
    reg [6:0]  tm      [0:2];     // remaining time 0..99
    reg [25:0] sec_cnt [0:2];     // 1-second prescaler per space
    reg [2:0]  expired;           // timer ran out and no coin since
 
    // Error ("selected but unoccupied") state
    reg        err_active;
    reg [2:0]  err_phase;         // 0..5 : on,off,on,off,on,off
    reg [24:0] err_tick;
    reg [1:0]  err_space;
 
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 3; i = i + 1) begin
                tm[i]      <= 7'd0;
                sec_cnt[i] <= 26'd0;
            end
            expired    <= 3'b000;
            err_active <= 1'b0;
            err_phase  <= 3'd0;
            err_tick   <= 25'd0;
            err_space  <= 2'd0;
        end else begin
            //---------------- timers ----------------
            for (i = 0; i < 3; i = i + 1) begin
                if (!occ[i]) begin
                    // vacant: clear everything
                    tm[i]      <= 7'd0;
                    sec_cnt[i] <= 26'd0;
                    expired[i] <= 1'b0;
                end else begin
                    // countdown
                    if (tm[i] == 7'd0)
                        sec_cnt[i] <= 26'd0;                 // hold: starts 1 s after next coin
                    else if (sec_cnt[i] == ONE_SEC-1) begin
                        sec_cnt[i] <= 26'd0;
                        tm[i]      <= tm[i] - 7'd1;
                        if (tm[i] == 7'd1) expired[i] <= 1'b1;
                    end else
                        sec_cnt[i] <= sec_cnt[i] + 26'd1;
 
                    // coin deposit (overrides the decrement above in the same cycle)
                    if (coin_pulse && sel_valid && (sel_idx == i)) begin
                        tm[i]      <= (tm[i] >= 7'd89) ? 7'd99 : (tm[i] + 7'd10);
                        expired[i] <= 1'b0;
                    end
                end
            end
 
            //---------------- error blink sequencer ----------------
            if (err_active) begin
                if (err_tick == ERR_HALF-1) begin
                    err_tick <= 25'd0;
                    if (err_phase == 3'd5) err_active <= 1'b0;
                    else                   err_phase  <= err_phase + 3'd1;
                end else
                    err_tick <= err_tick + 25'd1;
            end
 
            if (coin_pulse && sel_valid && !sel_occ && !err_active) begin
                err_active <= 1'b1;
                err_phase  <= 3'd0;
                err_tick   <= 25'd0;
                err_space  <= sel_idx;
            end
        end
    end
 
    //------------------------------------------------------------------
    // LED logic
    //------------------------------------------------------------------
    reg [2:0] led_space;
    integer j;
    always @(*) begin
        for (j = 0; j < 3; j = j + 1) begin
            if (err_active && (err_space == j))
                led_space[j] = ~err_phase[0];               // 3 blinks
            else if (!occ[j])
                led_space[j] = 1'b0;                        // vacant: off
            else if (tm[j] == 7'd0)
                led_space[j] = expired[j] ? blink_tenth : 1'b1;   // 0.1 s blink / steady
            else if (tm[j] <= 7'd10)
                led_space[j] = blink_half;                  // 0.5 s blink
            else
                led_space[j] = 1'b1;                        // steady on
        end
    end
 
    assign LEDR[2:0] = led_space;
    assign LEDR[7:3] = 5'b00000;
    assign LEDR[9:8] = SW[9:8];
 
    //------------------------------------------------------------------
    // Displays
    //------------------------------------------------------------------
    // Available spaces
    reg [1:0] avail;
    always @(*) avail = 2'd3 - occ[0] - occ[1] - occ[2];
 
    assign HEX5 = (avail != 0) ? SEG_O : SEG_F;
    assign HEX4 = (avail != 0) ? SEG_P : SEG_L;
    assign HEX3 = seg_digit({2'b00, avail});
 
    // Selected space
    assign HEX2 = sel_valid ? seg_digit({2'b00, sel}) : SEG_DASH;
 
    // Timer of selected space
    wire [6:0] sel_time = sel_valid ? tm[sel_idx] : 7'd0;
    wire [3:0] tens     = sel_time / 10;
    wire [3:0] ones     = sel_time % 10;
 
    assign HEX1 = !sel_valid ? SEG_DASH :
                  err_active ? SEG_E    : seg_digit(tens);
    assign HEX0 = !sel_valid ? SEG_DASH :
                  err_active ? SEG_R    : seg_digit(ones);
 
endmodule
