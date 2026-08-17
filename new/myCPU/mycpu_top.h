// 文件职责：集中定义CPU所有级间总线宽度，避免生产者和消费者使用不同切片。
// 上游：各流水级按照本文件的宏打包和解包控制/数据字段。
// 下游：mycpu_top和各stage共享这些宏；修改任意宽度都必须同步审查所有切片。
`ifndef MYCPU_TOP_H
    `define MYCPU_TOP_H

    // IF1 → IF2：IF1发出的双宽取指请求，IF2用它和下一拍IROM数据对齐。
    // IF1到IF2保存一个双宽取指请求：2位条数和两份71位预测元数据。
    `define IF_TO_IF2_BUS_WIDTH 144

    // IF2队列单条payload：指令、PC和预测元数据，不包含slot0额外RAS快照。
    // [阶段2新增] FIFO第二槽送往ID影子译码的只读总线宽度。
    // 它不包含在FIFO出口才为真实slot0拼接的3位RAS快照。
    `define FETCH_BUS_WIDTH 103

    // IF2 → ID：slot0在FETCH_BUS基础上追加3位RAS快照，保持旧IF_TO_ID名字兼容。
    `define IF_TO_ID_BUS_WIDTH 106

    // ID内部完整译码包和送ISSUE包。ID_TO_ISSUE最高位预留late/依赖标志。
    // [阶段5C修改] 原371位控制包保持最低位字段位置不变，在最高位增加
    // late/dep1/dep2三位，避免扰动ISSUE中的操作数切片和长数据路径。
    `define ID_DATA_BUS_WIDTH 371
    `define ID_TO_ISSUE_BUS_WIDTH 374

    // ID → ISSUE寄存器堆读地址：两个槽各带use_rs1/use_rs2和rs1/rs2编号。
    `define ID_TO_ISSUE_RF_BUS_WIDTH 24

    // ISSUE → EXE：在ID包基础上附加LW短旁路标签，EXE按标签修复ALU源操作数。
    // [阶段5C2修改] ISSUE在最高位再增加ld1/ld2两个短标签。它们只表示
    // 下一拍EXE的哪个源操作数应改用LW返回值，原374位控制包位置不变。
    `define ISSUE_TO_EXE_BUS_WIDTH 376
    // [兼容] EXE内部字段排列在阶段3保持不变，旧名字作为等宽别名保留。
    `define ID_TO_EXE_BUS_WIDTH `ISSUE_TO_EXE_BUS_WIDTH

    // ID → IF：RAS维护信息，只有CALL/RET类指令会让IF更新返回地址栈。
    `define ID_TO_IF_BUS_WIDTH 34

    // EXE → MEM：普通结果、访存元数据、MUL标记和slot1 late_alu元数据。
    // [阶段5C修改] 5A的is_mul仍在最低位；新增79位后置ALU元数据。
    `define EXE_TO_MEM_BUS_WIDTH 338

    // MEM → WB：写回结果、退休信息、访存追踪字段和trap/CSR控制字段。
    `define MEM_TO_WB_BUS_WIDTH 284   // 原总线 + 退休指令/dnpc/访存元数据和load读值

    // WB → ISSUE：写回旁路包，ISSUE用它覆盖寄存器堆旧值。
    `define WB_TO_ID_BUS_WIDTH 39

    // EXE → IF：分支纠错、BTB更新和RAS恢复指针。
    `define EXE_TO_IF_BUS_WIDTH 60

    // EXE/MEM → ISSUE：三级旁路网络的短包，按EXE优先、MEM次之、WB最后使用。
    // [阶段5C2修改] 在load与late之间增加is_lw。只有完整32位LW允许
    // 走EXE修复旁路，LB/LBU/LH/LHU仍保持原停顿规则。
    `define EXE_TO_ID_BYPASS_BUS_WIDTH 43
    `define MEM_TO_ID_BYPASS_BUS_WIDTH 39
`endif // MYCPU_TOP_H
