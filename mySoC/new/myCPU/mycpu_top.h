`ifndef MYCPU_TOP_H
    `define MYCPU_TOP_H
    `define IF_TO_ID_BUS_WIDTH 106
    `define ID_TO_EXE_BUS_WIDTH 335   // +26: inst_ecall/mret/6xCSR指令(8) + csr_addr(12) + csr_uimm(5) + is_csr_inst(1)
    `define ID_TO_IF_BUS_WIDTH 34
    `define EXE_TO_MEM_BUS_WIDTH 124   // +48: inst_ecall(1)+inst_mret(1)+csr_we(1)+csr_addr(12)+csr_wdata(32)+is_csr_inst(1)
    `define MEM_TO_WB_BUS_WIDTH 118   // +48: 透传 EXE 的 CSR 域
    `define WB_TO_ID_BUS_WIDTH 39
    `define EXE_TO_IF_BUS_WIDTH 60
    `define EXE_TO_ID_BYPASS_BUS_WIDTH 41
    `define MEM_TO_ID_BYPASS_BUS_WIDTH 39
`endif // MYCPU_TOP_H