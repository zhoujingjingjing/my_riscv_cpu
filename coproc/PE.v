module PE(
	// interface to system
    input wire CLK,                         // CLK = 200MHz
    input wire RESET,                       // RESET, Negedge is active
    input wire EN,                          // enable signal for the accelerator, high for active
    input wire SELECTOR,                    // weight select read or use
    input wire W_EN,                         // enable weight to flow
    // interface to PE row .....
    input wire signed[7:0]active_left,
    output reg signed[7:0]active_right,

    input wire signed[15:0]in_sum,
    output reg signed[15:0]out_sum,

    input wire signed[7:0]in_weight_above,
    output wire signed[7:0]out_weight_below
	);
    reg signed[7:0] weight_1;
    reg signed[7:0] weight_2;
    // [修复] 权重向下传递改为寄存器化(原来 out_weight_below 是 weight_1/2 的组合输出，
    //   叠加 always 块里 weight_1/2 用阻塞赋值 → 跨 PE 形成组合环竞争，整列权重在一拍内
    //   被同一个值冲刷，装载结果不确定)。改成专门的寄存器逐拍下移一行，彻底消除竞争。
    reg signed[7:0] weight_below_reg;

    // multiplier
    // accumulator (here use register to calculate and accumulate in one cycle)
    // registers for systolic dataflow
    always @(negedge RESET or posedge CLK )begin
        if(~RESET)
        begin
            out_sum <= 0;
            active_right <= 0;
            weight_1 <= 0;
            weight_2 <= 0;
            weight_below_reg <= 0;
        end
        else
        begin
            if(EN)
            begin
                active_right <= active_left;
                // 计算用当前选中的权重；装载写另一个缓冲，均改为非阻塞赋值
                if(SELECTOR)
                    out_sum <= weight_2*active_left+in_sum;
                else
                    out_sum <= weight_1*active_left+in_sum;
                if(W_EN)
                begin
                    if(SELECTOR) weight_1 <= in_weight_above;
                    else         weight_2 <= in_weight_above;
                    weight_below_reg <= in_weight_above;   // 寄存器化下传：一拍一行，无竞争
                end
            end
        end
    end
    assign out_weight_below = weight_below_reg;
endmodule