`ifndef MYCPU_TOP_H
    `define MYCPU_TOP_H
    `define IF_TO_ID_BUS_WIDTH 106
    // [RV32M移植] ID→EXE新增is_rv32m和3位funct3操作类型，共增加4位。
    `define ID_TO_EXE_BUS_WIDTH 371
    `define ID_TO_IF_BUS_WIDTH 34
    `define EXE_TO_MEM_BUS_WIDTH 258   // 原总线 + 退休指令/dnpc/访存元数据
    `define MEM_TO_WB_BUS_WIDTH 284   // 原总线 + 退休指令/dnpc/访存元数据和load读值
    `define WB_TO_ID_BUS_WIDTH 39
    `define EXE_TO_IF_BUS_WIDTH 60
    `define EXE_TO_ID_BYPASS_BUS_WIDTH 41
    `define MEM_TO_ID_BYPASS_BUS_WIDTH 39
`endif // MYCPU_TOP_H
