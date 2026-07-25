`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/16/2025 06:28:41 PM
// Design Name: 
// Module Name: tb_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module tb_top_old;
    reg clk;

    reg serial_rx;          
    wire serial_tx;         
    
    reg [7:0] rx_data[0:17];
    integer j;
    
    top uut (
        .i_sys_clk_p(clk),
        .i_sys_clk_n(~clk),
        .i_uart_rx(serial_rx),
        .o_uart_tx(serial_tx),
        .virtual_led(),  
        .virtual_seg()
    );
    

    //  clock 50MHz=2.5 20ns
    initial begin
        clk = 0;
        forever #2.5 clk = ~clk;
    end

    initial begin
        serial_rx = 1;
        #200;
    end

    task uart_send_byte(input [7:0] data);
        integer i;
        begin
            serial_rx = 0;  // start bit
            #(104166);      // baud 9600, 1bit = 1/9600s = 104166ns

            for(i = 0; i < 8; i = i + 1) begin
                serial_rx = data[i];
                #(104166);
            end

            serial_rx = 1;  // stop bit
            #(104166);
        end
    endtask

    task uart_receive_byte(output [7:0] data);
        integer i;
        begin
            // wait for start bit
            wait(serial_tx == 0);
            #(52083);  // half of a process

            for(i = 0; i < 8; i = i + 1) begin
                #(104166);
                data[i] = serial_tx;
            end

            #(104166); // stop bit
        end
    endtask

    // // ====================================================================
    // // 原汁原味的 CPU 探针与控制台打印逻辑
    // // ====================================================================
    // wire cpu_clk = uut.cpu_clk;
    
    // // 监听指令提交
    // wire wb_valid = uut.student_top_inst.Core_cpu.wb_stage.wb_valid;
    // wire [31:0] wb_pc      = uut.student_top_inst.Core_cpu.debug_wb_pc;
    // wire [3:0]  wb_rf_we   = uut.student_top_inst.Core_cpu.debug_wb_rf_we;
    // wire [4:0]  wb_rf_wnum = uut.student_top_inst.Core_cpu.debug_wb_rf_wnum;
    // wire [31:0] wb_rf_wdata= uut.student_top_inst.Core_cpu.debug_wb_rf_wdata;

    // // 监听内存写操作 (用来判定官方的 PASS / FAIL 测试点)
    // wire        data_en    = uut.student_top_inst.Core_cpu.data_sram_en;
    // wire [3:0]  data_we    = uut.student_top_inst.Core_cpu.data_sram_we;
    // wire [31:0] data_addr  = uut.student_top_inst.Core_cpu.data_sram_addr;
    // wire [31:0] data_wdata = uut.student_top_inst.Core_cpu.data_sram_wdata;

    // // 打印 Trace 到控制台（如果你觉得太慢卡死，可以把这三行注释掉）
    // always @(posedge cpu_clk) begin
    //     if (wb_valid) begin
    //         // $display("[CPU TRACE] Time: %0t | PC = 0x%8h | WE = %b | Reg[%2d] = 0x%8h",
    //         //          $time, wb_pc, (|wb_rf_we), wb_rf_wnum, wb_rf_wdata);
            
    //         // 跑到死循环结束点，自动停止仿真
    //         if (wb_pc == 32'h8000_0010) begin
    //              $display("==== Test Program Reached END_PC 0x80000010 ====");
    //              $finish;
    //         end
    //     end
    // end

    // // 监听官方测试用例的判定标记 (同步修改为 0x8010_XXXX 高位地址)
    // always @(posedge cpu_clk) begin
    //     if (data_en && data_we != 4'b0000) begin
    //         if (data_addr == 32'h8010_0000) begin
    //             $display("---- [%0t] Sub-test PASS! Passed Count = %0d ----", $time, data_wdata);
    //         end
    //         else if (data_addr == 32'h8010_0004) begin
    //             $display("==============================================================");
    //             $display(" FATAL ERROR!!! Sub-test FAILED! CPU wrote to FAIL Address 0x80100004");
    //             $display("==============================================================");
    //             $finish;
    //         end
    //     end
    // end
    // // ====================================================================

    initial begin
        #1000;
        // $timeformat(-9,0," ns",10); // 设置一下时间格式，让上面的 Trace 打印更好看
        $display("==== send 0x00 to uart_rx ====");
        uart_send_byte(8'h00);
        
        fork
            begin: RX_MONITOR
                wait(serial_tx == 0);
                $display("ERROR: 0x00 should not have tx data?");
                $finish;
            end
            begin: TIMEOUT
                #100000;
                disable RX_MONITOR;
                $display("PASS: 0x00 instruction");
            end
        join

        $display("==== send 0x81 SW[0]=1 ====");
        uart_send_byte(8'b10000001);
        #2000;
        
        $display("==== send 0xa0 SW[31]=1 ====");
        uart_send_byte(8'b10000001 + 31); 
        #2000;

        $display("==== send 0xc1 KEY[0]=1 ====");
        uart_send_byte(8'b10000000 + 65);
        #2000;

        $display("==== send 0x80 read 18bit data  ====");
        uart_send_byte(8'h80); 
          
        for(j = 0; j < 18; j = j + 1) begin
            uart_receive_byte(rx_data[j]);
            $display("RX[%0d] = %02x", j, rx_data[j]);
        end

        if(rx_data[5][0] !== 1'b1 || rx_data[6][0] != 1'b1 || rx_data[9][7] != 1'b1)
            $display("ERROR: SW[0] KEY[0] SW[31] data error");
        else
            $display("PASS: SW[0] KEY[0] SW[31] data right");
           
        $finish;
    end
endmodule
