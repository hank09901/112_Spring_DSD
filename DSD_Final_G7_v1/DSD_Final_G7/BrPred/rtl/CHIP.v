// Top module of your design, you cannot modify this module!!
module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;


//=========================================
	// Note that the overall design of your RISCV includes:
	// 1. pipelined RISCV processor
	// 2. data cache
	// 3. instruction cache


	RISCV_Pipeline i_RISCV(
		// control interface
		.clk            (clk)           , 
		.rst_n          (rst_n)         ,
//----------I cache interface-------		
		.ICACHE_ren     (ICACHE_ren)    ,
		.ICACHE_wen     (ICACHE_wen)    ,
		.ICACHE_addr    (ICACHE_addr)   ,
		.ICACHE_wdata   (ICACHE_wdata)  ,
		.ICACHE_stall   (ICACHE_stall)  ,
		.ICACHE_rdata   (ICACHE_rdata)  ,
//----------D cache interface-------
		.DCACHE_ren     (DCACHE_ren)    ,
		.DCACHE_wen     (DCACHE_wen)    ,
		.DCACHE_addr    (DCACHE_addr)   ,
		.DCACHE_wdata   (DCACHE_wdata)  ,
		.DCACHE_stall   (DCACHE_stall)  ,
		.DCACHE_rdata   (DCACHE_rdata)
	);
	

	cache D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (mem_read_D)  ,
        .mem_write  (mem_write_D) ,
        .mem_addr   (mem_addr_D)  ,
        .mem_wdata  (mem_wdata_D) ,
        .mem_rdata  (mem_rdata_D) ,
        .mem_ready  (mem_ready_D)
	);

	cache I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_wdata (ICACHE_wdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (mem_read_I)  ,
        .mem_write  (mem_write_I) ,
        .mem_addr   (mem_addr_I)  ,
        .mem_wdata  (mem_wdata_I) ,
        .mem_rdata  (mem_rdata_I) ,
        .mem_ready  (mem_ready_I)
	);
endmodule

module RISCV_Pipeline(
    clk, 
    rst_n,
    // for I Cache -------------------
    ICACHE_ren, 
    ICACHE_wen, 
    ICACHE_addr, 
    ICACHE_wdata, 
    ICACHE_stall, 
    ICACHE_rdata,
    // for D Cache --------------------
    DCACHE_ren, 
    DCACHE_wen, 
    DCACHE_addr, 
    DCACHE_wdata, 
    DCACHE_stall, 
    DCACHE_rdata
);
      
    
    input clk;
    input rst_n;
    // for I Cache -------------------
    output        ICACHE_ren; 
    output        ICACHE_wen; 
    output [29:0] ICACHE_addr; 
    output [31:0] ICACHE_wdata; 
    input         ICACHE_stall; 
    input  [31:0] ICACHE_rdata;
    // for D Cache --------------------
    output        DCACHE_ren; 
    output        DCACHE_wen; 
    output [29:0] DCACHE_addr; 
    output [31:0] DCACHE_wdata; 
    input         DCACHE_stall; 
    input  [31:0] DCACHE_rdata;
    
    // wire/reg declration--------------------------------
    reg           ICACHE_ren;
    reg           ICACHE_wen;
    reg    [29:0] ICACHE_addr;
    reg    [31:0] ICACHE_wdata;
    reg           DCACHE_ren;
    reg           DCACHE_wen;
    reg    [29:0] DCACHE_addr;
    reg    [31:0] DCACHE_wdata;
    
    reg    [31:0] IF_PC;
    wire   [31:0] IF_PC_nxt;
    reg    [31:0] IF_PC4;
    reg           IF_flush;
    reg           IF_PCSrc;
    reg    [31:0] IF_immediate;
    reg    [31:0] IF_branch_address;
    wire          IF_B;
    wire          IF_BrPred;
    reg    [31:0] IF_branch_choose_address;
    
    reg           ID_BrPred, ID_BrPred_nxt;
    reg    [31:0] ID_PC, ID_PC_nxt;
    reg    [31:0] ID_PC4, ID_PC4_nxt;
    reg    [31:0] ID_instruction, ID_instruction_nxt;
    wire   [31:0] ID_instruction_wire;
    wire   [6:0]  ID_opcode;
    wire   [6:0]  ID_funct7;
    wire   [2:0]  ID_funct3;
    reg           ID_flush;
    wire          ID_flush_nxt;
    wire   [4:0]  ID_rs1, ID_rs2, ID_rd;
    wire   [31:0] ID_rdata1, ID_rdata2; 
    wire   [31:0] ID_wdata;
    wire   [31:0] ID_immediate;
    wire          ID_RegWrite;
    wire          ID_ALUSrc1;
    wire   [1:0]  ID_ALUSrc2;
    wire   [1:0]  ID_ALUOP;
    wire          ID_MemWrite;
    wire          ID_MemtoReg;
    wire          ID_MemRead;
    wire   [1:0]  ID_Branch;
    wire          ID_jump;
    wire          ID_Jalr;
    wire          ID_branch_jump;
    wire   [2:0]  ID_branch_ctrl1, ID_branch_ctrl2;
    wire          ID_hazard;
    reg    [31:0] ID_jump_address;
    wire   [31:0] ID_jump_rdata;
    reg    [31:0] ID_branch_address, ID_branch_address_nxt;
    reg    [31:0] ID_branch_choose_address;
    reg           ID_predict;
    wire          ID_predict_wire;
    reg           ID_B, ID_B_nxt;
    wire          ID_B_wire;
    wire          ID_nop;
    
    reg           IF_PC_stall;
    reg           IF_ID_stall;    
    reg           ID_EX_stall;    
    reg           EX_MEM_stall;
    reg           MEM_WB_stall;
    
    
    
    
    reg    [31:0] EX_PC, EX_PC_nxt;
    reg    [6:0]  EX_funct7, EX_funct7_nxt;
    reg    [2:0]  EX_funct3, EX_funct3_nxt;
    wire   [6:0]  EX_funct7_wire;
    wire   [2:0]  EX_funct3_wire;
    wire   [3:0]  EX_alu_operation;
    reg    [4:0]  EX_rs1, EX_rs2, EX_rd;
    reg    [4:0]  EX_rs1_nxt, EX_rs2_nxt, EX_rd_nxt;
    wire   [4:0]  EX_rd_wire, EX_rs1_wire, EX_rs2_wire;
    reg    [31:0] EX_rdata1, EX_rdata2, EX_wdata;
    wire   [31:0] EX_rdata1_nxt, EX_rdata2_nxt;
    reg    [31:0] EX_wdata_nxt;
    reg    [31:0] EX_immediate, EX_immediate_nxt;
    reg           EX_RegWrite, EX_RegWrite_nxt;
    wire          EX_RegWrite_wire;
    reg           EX_ALUSrc1, EX_ALUSrc1_nxt;
    reg    [1:0]  EX_ALUSrc2, EX_ALUSrc2_nxt;
    reg    [1:0]  EX_ALUOP, EX_ALUOP_nxt;
    wire   [1:0]  EX_ALUOP_wire;
    reg           EX_MemWrite, EX_MemWrite_nxt;
    reg           EX_MemtoReg, EX_MemtoReg_nxt;
    reg           EX_MemRead, EX_MemRead_nxt;
    wire          EX_MemRead_wire;
    reg    [1:0]  EX_Branch, EX_Branch_nxt;
    reg           EX_jump, EX_jump_nxt;
    reg           EX_Jalr, EX_Jalr_nxt;
    reg    [31:0] EX_forwardA_data, EX_forwardB_data;
    wire   [31:0] EX_alu_data1, EX_alu_data2;
    
    wire   [31:0] EX_alu_result;
    wire   [1:0]  ForwardA, ForwardB;
    
    
    reg    [31:0] MEM_PC, MEM_PC_nxt;
    reg    [4:0]  MEM_rd, MEM_rd_nxt;
    wire   [4:0]  MEM_rd_wire;
    reg           MEM_RegWrite, MEM_RegWrite_nxt;
    wire          MEM_RegWrite_wire;
    reg           MEM_MemWrite, MEM_MemWrite_nxt;
    reg           MEM_MemtoReg, MEM_MemtoReg_nxt;
    reg           MEM_MemRead,  MEM_MemRead_nxt;
    wire          MEM_MemRead_wire;
    reg    [31:0] MEM_DCACHE_wdata, MEM_DCACHE_wdata_nxt;
    wire   [31:0] MEM_DCACHE_rdata;
    reg    [31:0] MEM_alu_result, MEM_alu_result_nxt;
    wire   [31:0] MEM_alu_result_wire;
    reg    [4:0]  WB_rd, WB_rd_nxt;
    wire   [4:0]  WB_rd_wire;
    wire          WB_RegWrite_wire;
    reg           WB_RegWrite, WB_RegWrite_nxt;
    reg           WB_MemtoReg, WB_MemtoReg_nxt;
    reg    [31:0] WB_alu_result, WB_alu_result_nxt;
    reg    [31:0] WB_DCACHE_rdata, WB_DCACHE_rdata_nxt;
    wire   [31:0] WB_regfile_wdata;
    
    reg    [1:0]  pred_state, pred_state_nxt;
    wire          branch_stall;
    // submodule---------------------------------------------------------
    
    
    Ctrl Control(
        .opcode(ID_opcode),
        .funct3(ID_funct3),
        .RegWrite(ID_RegWrite), 
        .ALUSrc1(ID_ALUSrc1),
        .ALUSrc2(ID_ALUSrc2), 
        .ALUOP(ID_ALUOP), 
        .MemWrite(ID_MemWrite), 
        .MemtoReg(ID_MemtoReg), 
        .MemRead(ID_MemRead), 
        .Branch(ID_Branch), 
        .Jump(ID_jump), 
        .Jalr(ID_Jalr),
        .nop(ID_nop)
    );
    
    
    reg_file RegFile(
        .clk(clk), 
        .rst_n(rst_n), 
        .wen(WB_RegWrite_wire), 
        .reg1(ID_rs1), 
        .reg2(ID_rs2), 
        .regw(WB_rd_wire), 
        .write(WB_regfile_wdata), 
        .read1(ID_rdata1), 
        .read2(ID_rdata2)
    );
    
    ImmGen ImmediteGenerator(.instruction(ID_instruction_wire), .immediate(ID_immediate));
    
    ALUCtrl ALUContorl(.ALUOP(EX_ALUOP_wire), .funct7(EX_funct7_wire), .funct3(EX_funct3_wire), .ALU_operation(EX_alu_operation));
    
    ALU ALU_Unit(.data1(EX_alu_data1), .data2(EX_alu_data2), .ALU_operation(EX_alu_operation), .result(EX_alu_result));
    
    Hazard_Detection Hazard_detection_unit(.hazard(ID_hazard), .EX_MemRead(EX_MemRead_wire), .EX_rd(EX_rd_wire), .ID_rs1(ID_rs1), .ID_rs2(ID_rs2));
    
    Forward forward_unit(
        .ForwardA(ForwardA), 
        .ForwardB(ForwardB), 
        .EX_rs1(EX_rs1_wire), 
        .EX_rs2(EX_rs2_wire), 
        .MEM_rd(MEM_rd_wire), 
        .WB_rd(WB_rd_wire), 
        .MEM_RegWrite(MEM_RegWrite_wire), 
        .WB_RegWrite(WB_RegWrite_wire)
    );
    
    Forward_Branch forward_branch_unit(
        .branch_ctrl1(ID_branch_ctrl1), 
        .branch_ctrl2(ID_branch_ctrl2), 
        .ID_rs1(ID_rs1), 
        .ID_rs2(ID_rs2), 
        .EX_rd(EX_rd_wire), 
        .EX_RegWrite(EX_RegWrite_wire), 
        .MEM_rd(MEM_rd_wire), 
        .MEM_RegWrite(MEM_RegWrite_wire), 
        .MEM_MemRead(MEM_MemRead_wire), 
        .WB_rd(WB_rd_wire), 
        .WB_RegWrite(WB_RegWrite_wire)
    );
    
    Branch_Cmp Branch_Comparator(
        .branch_jump(ID_branch_jump), 
        .jump_read_data(ID_jump_rdata), 
        .Branch(ID_Branch), 
        .branch_ctrl1(ID_branch_ctrl1), 
        .branch_ctrl2(ID_branch_ctrl2), 
        .ID_rdata1(ID_rdata1), 
        .ID_rdata2(ID_rdata2), 
        .EX_alu_result(EX_alu_result), 
        .MEM_rdata(MEM_DCACHE_rdata), 
        .MEM_alu_result(MEM_alu_result), 
        .WB_wdata(WB_regfile_wdata)
    );
    
    Predict Predict_Unit(
        .clk(clk), 
        .rst_n(rst_n), 
        .IF_B(IF_B), 
        .ID_B(ID_B_wire), 
        .predict(ID_predict_wire), 
        .stall(branch_stall), 
        .BrPred(IF_BrPred)
    );
    
    // continuous assignment---------------------------------------------
    assign IF_B = (!(ICACHE_rdata[30:24] ^ 7'b1100011))? 1'b1 : 1'b0;  // see whether it's branch inst or not
    assign ID_opcode = ID_instruction[6:0];
    assign ID_funct7 = ID_instruction[31:25];
    assign ID_funct3 = ID_instruction[14:12];
    assign ID_rs1 = ID_instruction[19:15];
    assign ID_rs2 = ID_instruction[24:20];
    assign ID_rd  = ID_instruction[11:7];
    assign ID_nop = (!(ID_instruction ^ 32'b000000000000_00000_000_00000_0010011))? 1'b1:1'b0;
    assign WB_RegWrite_wire = WB_RegWrite;
    assign ID_instruction_wire = ID_instruction;
    assign EX_ALUOP_wire = EX_ALUOP;
    assign EX_funct7_wire = EX_funct7;
    assign EX_funct3_wire = EX_funct3;
    assign EX_MemRead_wire = EX_MemRead;
    assign EX_rd_wire = EX_rd;
    assign EX_rs1_wire = EX_rs1;
    assign EX_rs2_wire = EX_rs2;
    assign EX_alu_data1 = (EX_ALUSrc1)? EX_PC : EX_forwardA_data;
    assign EX_alu_data2 = (EX_ALUSrc2[1])? 32'd4 : ((EX_ALUSrc2[0])? EX_immediate : EX_forwardB_data);
    assign MEM_rd_wire = MEM_rd;
    assign WB_rd_wire = WB_rd;
    assign MEM_RegWrite_wire = MEM_RegWrite;
    assign EX_RegWrite_wire = EX_RegWrite;
    assign MEM_MemRead_wire = MEM_MemRead;
    assign MEM_DCACHE_rdata = {DCACHE_rdata[7:0], DCACHE_rdata[15:8], DCACHE_rdata[23:16], DCACHE_rdata[31:24]};
    assign MEM_alu_result_wire = MEM_alu_result;
    assign WB_regfile_wdata = (WB_MemtoReg)?  WB_DCACHE_rdata : WB_alu_result;
    assign branch_stall = (ICACHE_stall||DCACHE_stall)? 1: (ID_hazard)? 1:0;
    // PC DFF-----------------------------------------------------------
    always @(posedge clk) begin
       if(!rst_n) begin
           IF_PC <= 32'b0;
       end
       else if(IF_PC_stall) begin
           IF_PC <= IF_PC;
       end
       else begin
           IF_PC <= IF_PC_nxt;
       end
    end
    
    // IF stage --------------------------------------------------------------
    assign ID_flush_nxt = IF_flush;
    assign IF_PC_nxt = (ID_jump)? ID_jump_address : IF_branch_choose_address;  // Whether to jump_branch or not
    always @(*) begin
        IF_PCSrc = ID_jump || ID_branch_jump;
        IF_PC4 = IF_PC + 32'd4;
        ICACHE_ren = 1'b1;
        ICACHE_wen = 1'b0;
        ICACHE_addr = IF_PC[31:2];
        ICACHE_wdata = 32'b0;

        IF_flush = (ID_jump || !(ID_predict)) && (!ID_flush);  // if it's jump instruction or branch prediction is false
        IF_immediate = {{20{ICACHE_rdata[7]}} ,ICACHE_rdata[31],ICACHE_rdata[6:1],ICACHE_rdata[19:16],1'b0};
        IF_branch_address = IF_PC + IF_immediate;
        
        
        if(ID_predict) begin
            if(IF_BrPred) IF_branch_choose_address = (IF_B)? IF_branch_address : IF_PC4;
            else IF_branch_choose_address = IF_PC4;
        end
        else begin
            if(ID_nop) IF_branch_choose_address = (!IF_BrPred)? IF_PC4 : ((IF_B)? IF_branch_address : IF_PC4);
            else IF_branch_choose_address = ID_branch_choose_address;
        end
        
        
        ID_PC_nxt = IF_PC;
        ID_PC4_nxt = IF_PC4;
        ID_instruction_nxt = {ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};
        ID_BrPred_nxt = IF_BrPred;
        ID_branch_address_nxt = IF_branch_address;
        ID_B_nxt = IF_B;
        
        IF_PC_stall  = ID_hazard || ICACHE_stall || DCACHE_stall;
        IF_ID_stall  = IF_PC_stall;
        ID_EX_stall  = (ICACHE_stall || DCACHE_stall);
        EX_MEM_stall = ID_EX_stall;
        MEM_WB_stall = ID_EX_stall; 
        
    end
    
    // IF/ID register -------------------------------------------------------
    always @(posedge clk) begin
        if(!rst_n) begin
            ID_PC <= 32'b0;
            ID_PC4 <= 32'b0;
            ID_instruction <= 32'b0;
            ID_flush <= 0;
            ID_BrPred <= 0;
            ID_branch_address <= 0;
            ID_B <= 0;
        end
        else if(IF_ID_stall) begin 
            ID_PC<=ID_PC;
            ID_PC4 <= ID_PC4;
            ID_instruction<=ID_instruction;
            ID_flush<=ID_flush;
            ID_BrPred <= ID_BrPred;
            ID_branch_address <= ID_branch_address;
            ID_B <= ID_B;
        end
        else begin
            if(IF_flush) ID_instruction <= {25'd0,7'b0010011};
            else ID_instruction <= ID_instruction_nxt;
            
            ID_PC <= ID_PC_nxt;
            ID_PC4 <= ID_PC4_nxt;
            ID_flush <= ID_flush_nxt;
            ID_BrPred <= ID_BrPred_nxt;
            ID_branch_address <= ID_branch_address_nxt;
            ID_B <= ID_B_nxt;
        end
    end
    
    // ID stage ---------------------------------------------------------------
    assign ID_predict_wire = ID_predict;
    assign ID_B_wire = ID_B;
    always @(*) begin 
        
        ID_jump_address = (ID_Jalr)? (ID_jump_rdata + ID_immediate) : (ID_PC + ID_immediate);  // jump address determined by Jalr or Btype&Ja
        
        if(!ID_B) begin // if it's not branch instruction, don't need to care prediction
            ID_predict = 1;
        end
        else begin  // if it's branch instruction
            if(ID_branch_jump == ID_BrPred) ID_predict = 1;
            else ID_predict = 0;
        end
        ID_branch_choose_address = (ID_BrPred)? ID_PC4 : ID_branch_address;
        
        EX_PC_nxt = ID_PC;
        EX_funct7_nxt = ID_funct7;
        EX_funct3_nxt = ID_funct3;
        EX_rs1_nxt = ID_rs1;
        EX_rs2_nxt = ID_rs2;
        EX_rd_nxt  = ID_rd;
        
        
        
        
        EX_wdata_nxt  = ID_wdata;
        EX_immediate_nxt = ID_immediate;
        EX_RegWrite_nxt = ID_RegWrite;
        EX_ALUSrc1_nxt = ID_ALUSrc1;
        EX_ALUSrc2_nxt = ID_ALUSrc2;
        EX_ALUOP_nxt = ID_ALUOP;
        EX_MemWrite_nxt = ID_MemWrite;
        EX_MemtoReg_nxt = ID_MemtoReg;
        EX_MemRead_nxt = ID_MemRead;
        EX_Branch_nxt = ID_Branch;
        EX_jump_nxt = ID_jump;
        EX_Jalr_nxt = ID_Jalr;
    end
    reg ID_foward_3cycle_A, ID_foward_3cycle_B;
    assign EX_rdata1_nxt = (ID_foward_3cycle_A)?WB_regfile_wdata:ID_rdata1;
    assign EX_rdata2_nxt = (ID_foward_3cycle_B)?WB_regfile_wdata:ID_rdata2;
    
    //3 cycle data hazzard forwarding
    always@(*)
    begin
        if( !(WB_rd^ID_rs1)&& (WB_RegWrite) && (|(WB_rd))  )
            ID_foward_3cycle_A=1;
        else
            ID_foward_3cycle_A=0;
        if( !(WB_rd^ID_rs2)&& (WB_RegWrite) && (|(WB_rd)) )
            ID_foward_3cycle_B=1;
        else
            ID_foward_3cycle_B=0;
    end
    
    
    // ID/EX register
    always @(posedge clk) begin
        if(!rst_n) begin
            EX_PC <= 32'b0;
            EX_funct7 <= 7'b0;
            EX_funct3 <= 3'b0;
            EX_rs1 <= 5'b0;
            EX_rs2 <= 5'b0;
            EX_rd  <= 5'b0;
            EX_rdata1 <= 32'b0;
            EX_rdata2 <= 32'b0;
            EX_wdata  <= 32'b0;
            EX_immediate <= 32'b0;
            EX_RegWrite <= 0;
            EX_ALUSrc1 <= 1'b0;
            EX_ALUSrc2 <= 2'b0;
            EX_ALUOP <= 2'b11;
            EX_MemWrite <= 0;
            EX_MemtoReg <= 0;
            EX_MemRead <= 0;
            EX_Branch <= 2'b0;
            EX_jump <= 0;
            EX_Jalr <= 0;
        end
        else if(ID_EX_stall) begin
            EX_PC <= EX_PC;
            EX_funct7 <= EX_funct7;
            EX_funct3 <= EX_funct3;
            EX_rs1 <= EX_rs1;
            EX_rs2 <= EX_rs2;
            EX_rd  <= EX_rd;
            EX_rdata1 <= EX_rdata1;
            EX_rdata2 <= EX_rdata2;
            EX_wdata  <= EX_wdata;
            EX_immediate <= EX_immediate;
            EX_RegWrite <= EX_RegWrite;
            EX_ALUSrc1 <= EX_ALUSrc1;
            EX_ALUSrc2 <= EX_ALUSrc2;
            EX_ALUOP <= EX_ALUOP;
            EX_MemWrite <= EX_MemWrite;
            EX_MemtoReg <= EX_MemtoReg;
            EX_MemRead <= EX_MemRead;
            EX_Branch <= EX_Branch;
            EX_jump <= EX_jump;
            EX_Jalr <= EX_Jalr;
        end
        else begin
            EX_PC <= EX_PC_nxt;
            EX_funct7 <= EX_funct7_nxt;
            EX_funct3 <= EX_funct3_nxt;
            EX_rs1 <= EX_rs1_nxt;
            EX_rs2 <= EX_rs2_nxt;
            EX_rd  <= EX_rd_nxt;
            EX_rdata1 <= EX_rdata1_nxt;
            EX_rdata2 <= EX_rdata2_nxt;
            EX_wdata  <= EX_wdata_nxt;
            EX_immediate <= EX_immediate_nxt;
            EX_RegWrite <= EX_RegWrite_nxt;
            EX_ALUSrc1 <= EX_ALUSrc1_nxt;
            EX_ALUSrc2 <= EX_ALUSrc2_nxt;
            EX_ALUOP <= EX_ALUOP_nxt;
            EX_MemWrite <= EX_MemWrite_nxt;
            EX_MemtoReg <= EX_MemtoReg_nxt;
            EX_MemRead <= EX_MemRead_nxt;
            EX_Branch <= EX_Branch_nxt;
            EX_jump <= EX_jump_nxt;
            EX_Jalr <= EX_Jalr_nxt;
        end
    end
    
    // EX stage
    always @(*) begin
        case(ForwardA)
            2'b00: EX_forwardA_data = EX_rdata1;
            2'b01: EX_forwardA_data = MEM_alu_result;
            2'b10: EX_forwardA_data = WB_regfile_wdata;
            2'b11: EX_forwardA_data = EX_rdata1;
        endcase

        case(ForwardB)
            2'b00: EX_forwardB_data = EX_rdata2;
            2'b01: EX_forwardB_data = MEM_alu_result;
            2'b10: EX_forwardB_data = WB_regfile_wdata;
            2'b11: EX_forwardB_data = EX_rdata2;
        endcase    
        
        MEM_rd_nxt = EX_rd;
        MEM_RegWrite_nxt = EX_RegWrite;
        MEM_MemWrite_nxt = EX_MemWrite;
        MEM_MemtoReg_nxt = EX_MemtoReg;
        MEM_MemRead_nxt  = EX_MemRead;
        MEM_alu_result_nxt = EX_alu_result;
        MEM_DCACHE_wdata_nxt = EX_forwardB_data;
    end

    // EX/MEM register
    always @(posedge clk) begin
        if(!rst_n) begin
            MEM_rd <= 0;
            MEM_RegWrite <= 0;
            MEM_MemWrite <= 0;
            MEM_MemtoReg <= 0;
            MEM_MemRead  <= 0;
            MEM_alu_result <= 32'b0;
            MEM_DCACHE_wdata <= 32'b0;
        end
        else if(EX_MEM_stall) begin
            MEM_rd <= MEM_rd;
            MEM_RegWrite <= MEM_RegWrite;
            MEM_MemWrite <= MEM_MemWrite;
            MEM_MemtoReg <= MEM_MemtoReg;
            MEM_MemRead  <=  MEM_MemRead;
            MEM_alu_result <= MEM_alu_result;
            MEM_DCACHE_wdata <= MEM_DCACHE_wdata;
        end
        else begin
            MEM_rd <= MEM_rd_nxt;
            MEM_RegWrite <= MEM_RegWrite_nxt;
            MEM_MemWrite <= MEM_MemWrite_nxt;
            MEM_MemtoReg <= MEM_MemtoReg_nxt;
            MEM_MemRead  <=  MEM_MemRead_nxt;
            MEM_alu_result <= MEM_alu_result_nxt;
            MEM_DCACHE_wdata <= MEM_DCACHE_wdata_nxt;
        end
    end
    
    
    // MEM stage
    always @(*) begin
        DCACHE_ren  = MEM_MemRead;
        DCACHE_wen  = MEM_MemWrite;
        DCACHE_addr = MEM_alu_result[31:2];
        DCACHE_wdata = {MEM_DCACHE_wdata[7:0], MEM_DCACHE_wdata[15:8], MEM_DCACHE_wdata[23:16], MEM_DCACHE_wdata[31:24]};
        
        WB_rd_nxt = MEM_rd;
        WB_RegWrite_nxt = MEM_RegWrite;
        WB_MemtoReg_nxt = MEM_MemtoReg;
        WB_alu_result_nxt = MEM_alu_result;
        WB_DCACHE_rdata_nxt = MEM_DCACHE_rdata;
    end
    
    
    // MEM/WB register
    always @(posedge clk) begin
        if(!rst_n) begin
            WB_rd <= 5'b0;
            WB_RegWrite <= 0;
            WB_MemtoReg <= 0;
            WB_alu_result <= 32'b0;
            WB_DCACHE_rdata <= 32'b0;
        end
        else if(MEM_WB_stall) begin
            WB_rd <= WB_rd;
            WB_RegWrite <= WB_RegWrite;
            WB_MemtoReg <= WB_MemtoReg;
            WB_alu_result <= WB_alu_result;
            WB_DCACHE_rdata <= WB_DCACHE_rdata;
        end
        else begin
            WB_rd <= WB_rd_nxt;
            WB_RegWrite <= WB_RegWrite_nxt;
            WB_MemtoReg <= WB_MemtoReg_nxt;
            WB_alu_result <= WB_alu_result_nxt;
            WB_DCACHE_rdata <= WB_DCACHE_rdata_nxt;
        end
    end
    
    // WB stage
    
    
endmodule

module Ctrl(opcode, funct3, RegWrite, ALUSrc1, ALUSrc2, ALUOP, MemWrite, MemtoReg, MemRead, Branch, Jump, Jalr, nop);
    input [6:0] opcode;
    input [2:0] funct3;
    input nop;
    output Jalr, Jump, MemRead, MemtoReg, MemWrite, ALUSrc1, RegWrite;
    output [1:0] Branch, ALUSrc2, ALUOP;

    reg Jalr, Jump, MemRead, MemtoReg, MemWrite, ALUSrc1, RegWrite;
    reg [1:0] Branch, ALUSrc2, ALUOP;
    
    parameter Rtype = 7'b0110011;
    parameter Itype = 7'b0010011;
    parameter sw    = 7'b0100011;
    parameter lw    = 7'b0000011;
    parameter Btype = 7'b1100011;  // beq, bne
    parameter jal   = 7'b1101111;
    parameter jalr  = 7'b1100111;

    always @(*) begin
        MemRead = 1'b0;
        MemtoReg = 1'b0;
        ALUSrc1 = 1'b0;
        ALUSrc2 = 2'b0;
        RegWrite = 1'b0;
        MemWrite = 1'b0;
        Branch = 2'b0;  
        Jump = 1'b0;
        Jalr = 1'b0;
        ALUOP = 2'b0;
        
        case(opcode)
            Rtype: begin
                ALUSrc1 = 0;
                ALUSrc2 = 0;
                MemtoReg = 0;
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 2'b0;
                ALUOP = 2'b10;
                Jump = 0;
                Jalr = 0;
            end
            Itype: begin
                ALUSrc1 = 0;
                ALUSrc2 = 2'b01;
                MemtoReg = 0;
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 2'b0;
                ALUOP = 2'b10;
                Jump = 0;
                Jalr = 0;
            end
            sw: begin
                ALUSrc1 = 0;
                ALUSrc2 = 2'b01;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 1;
                Branch = 2'b0;
                ALUOP = 2'b00;
                Jump = 0;
                Jalr = 0;
            end
            lw: begin
                ALUSrc1 = 0;
                ALUSrc2 = 2'b01;
                MemtoReg = 1; //from Memory_Data
                RegWrite = 1;
                MemRead = 1;
                MemWrite = 0;
                Branch = 2'b0;
                ALUOP = 2'b00;
                Jump = 0;
                Jalr = 0;
            end
            Btype: begin
                ALUSrc1 = 0;
                ALUSrc2 = 2'b00;
                MemtoReg = 0;
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                ALUOP = 2'b01;
                Jump = 0;
                Jalr = 0;
                case(funct3)
                    3'b000: Branch = 2'b01;  //beq
                    3'b001: Branch = 2'b10;  //bne
                    default: Branch = 2'b00;
                endcase
            end
            jal: begin
                ALUSrc1 = 1;
                ALUSrc2 = 2'b10;
                MemtoReg = 0;
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                Branch = 0;
                ALUOP = 2'b00;
                Jump = 1;
                Jalr = 0;
            end
            jalr: begin
                ALUSrc1 = 1;
                ALUSrc2 = 2'b10;
                MemRead = 0;
                MemtoReg = 0;
                RegWrite = 1;
                MemWrite = 0;
                Branch = 0;
                ALUOP = 2'b00;
                Jump = 1;
                Jalr = 1;
            end
        endcase
        if(nop == 1) begin
            ALUSrc1 = 0;
            ALUSrc2 = 2'b00;
            MemRead = 0;
            MemtoReg = 0;
            RegWrite = 0;
            MemWrite = 0;
            Branch = 0;
            ALUOP = 2'b00;
            Jump = 0;
            Jalr = 0;
        end
    end
    
endmodule

module ALUCtrl(ALUOP, funct7, funct3, ALU_operation);
    input  [1:0] ALUOP;
    input  [6:0] funct7;
    input  [2:0] funct3;
    output [3:0] ALU_operation;
    reg    [3:0] ALU_operation;
    
    always @(*) begin  // and => 4'b0000,  or => 4'b0001,  add => 4'b0010,  xor => 4'b0011,  sll => 4'b0100,  srli => 4'b0101,  sub => 4'b0110,  srai => 4'b0111, slt => 4'b1000
        ALU_operation = 4'b1111;
        case(ALUOP)
            2'b00: ALU_operation = 4'b0010;  //add for lw/sw
            
            2'b01: ALU_operation = 4'b0110;  //sub for beq/bne
            
            2'b10: begin  // for Rtype & Itype
                case(funct3)
                    3'b000: begin
                        if(!(funct7 ^ 7'b0100000)) ALU_operation = 4'b0110;  // sub
                        else ALU_operation = 4'b0010;  // add, addi
                    end
                    3'b001: ALU_operation = 4'b0100;  // sll, slli
                    3'b010: ALU_operation = 4'b1000;  // slt, slti
                    3'b100: ALU_operation = 4'b0011;  // xor, xori
                    3'b101: begin
                        case(funct7)
                            7'b0000000: ALU_operation = 4'b0101;  // srli
                            7'b0100000: ALU_operation = 4'b0111;  // srai
                            default:    ALU_operation = 4'b1111;
                        endcase
                    end
                    3'b110:  ALU_operation = 4'b0001;  // or, ori
                    3'b111:  ALU_operation = 4'b0000;  // and, andi
                    default: ALU_operation = 4'b1111;
                endcase
            end
            2'b11: begin 
                ALU_operation = 4'b1111;
            end
        endcase
    end
endmodule

module ALU(data1, data2, ALU_operation, result);
    input  signed [31:0] data1, data2;
    input  [3:0]  ALU_operation;
    output signed [31:0] result;
    reg    signed [31:0] result, subtraction;
    reg    carry;
    always @(*) begin
        {carry, subtraction} = data1 - data2; 
        case(ALU_operation)
            4'b0000: result = data1 & data2;           // and
            4'b0001: result = data1 | data2;           // or
            4'b0010: {carry, result} = data1 + data2;  // add
            4'b0011: result = data1 ^ data2;           // xor
            4'b0100: result = data1 << data2;          // sll
            4'b0101: result = data1 >> data2;          // srli 
            4'b0110: {carry, result} = data1 - data2;  // sub
            4'b0111: result = data1 >>> data2;         // srai
            4'b1000: result = {31'b0, subtraction[31]};// slt
            default: result = 32'b0;
        endcase
    end
    
endmodule

module ImmGen(instruction, immediate);
    input  [31:0] instruction;
    output [31:0] immediate;
    reg    [31:0] immediate;
    reg    [6:0]  opcode;
    reg    [2:0]  funct3;
    
    parameter JAL   = 7'b1101111;
    parameter JALR  = 7'b1100111;
    parameter Btype   = 7'b1100011;
    parameter SW    = 7'b0100011;
    parameter LW    = 7'b0000011;
    parameter Itype = 7'b0010011;
    
    always @(*) begin
        opcode = instruction[6:0];
        funct3 = instruction[14:12];
        case(opcode)
            JAL:   immediate ={{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
            JALR:  immediate = {{21{instruction[31]}}, instruction[30:20]};
            Btype: immediate = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            SW:    immediate = {{21{instruction[31]}}, instruction[30:25], instruction[11:7]};
            LW:    immediate = {{21{instruction[31]}}, instruction[30:20]};
            Itype: begin
                case(funct3)
                    3'b001:  immediate = {27'b0, instruction[24:20]};
                    3'b101:  immediate = {27'b0, instruction[24:20]};
                    default: immediate = {{21{instruction[31]}}, instruction[30:20]};
                endcase
            end
            default: immediate = 32'b0;
        endcase
    end
endmodule

module Forward(ForwardA, ForwardB, EX_rs1, EX_rs2, MEM_rd, WB_rd, MEM_RegWrite, WB_RegWrite);
    output [1:0] ForwardA;
    output [1:0] ForwardB;
    input  [4:0] EX_rs1, EX_rs2;
    input  [4:0] MEM_rd, WB_rd;
    input  MEM_RegWrite, WB_RegWrite;
    reg    [1:0] ForwardA, ForwardB;
    
    always @(*) begin
        /*
        if(MEM_RegWrite && (|MEM_rd)) begin
            if(!(MEM_rd ^ EX_rs1)) ForwardA = 2'b10;
            else if(WB_RegWrite && (|WB_rd) && !(WB_rd ^ EX_rs1)) ForwardA = 2'b01;
            else ForwardA = 2'b00;
            
            if(!(MEM_rd ^ EX_rs2)) ForwardB = 2'b10;
            else if(WB_RegWrite && (|WB_rd) && !(WB_rd ^ EX_rs2)) ForwardB = 2'b01;
            else ForwardB = 2'b00;
        end
        else begin
            ForwardA = 2'b00;
            ForwardB = 2'b00;
        end
        */
        
        if((MEM_RegWrite) && (!(MEM_rd  ^EX_rs1)) && (|(MEM_rd)) )
            ForwardA=2'd1;
        else if ( (WB_RegWrite) && (!(WB_rd^EX_rs1)) && (|( WB_rd)) )
            ForwardA=2'd2;
        else
            ForwardA=2'd0;
            
            
        if((MEM_RegWrite) && (!(MEM_rd^EX_rs2)) && (|(MEM_rd)) )
            ForwardB=2'd1;
        else if ( (WB_RegWrite) && (!(WB_rd^EX_rs2)) && (|( WB_rd)) )
            ForwardB=2'd2;
        else
            ForwardB=2'd0;
    end
    
endmodule

module Hazard_Detection(hazard, EX_MemRead, EX_rd, ID_rs1, ID_rs2);
    output hazard;
    input EX_MemRead;
    input [4:0] EX_rd;
    input [4:0] ID_rs1, ID_rs2;
    
    reg hazard;
    
    always @(*) begin
        hazard = 1'b0;
        if(EX_MemRead) begin  // is lw instruction
            if(|(EX_rd)) begin  //  rd != 0
                if((!(EX_rd ^ ID_rs1)) || (!(EX_rd ^ ID_rs2))) begin
                    hazard = 1'b1;
                end
            end
        end
    end
    
endmodule

module Forward_Branch(branch_ctrl1, branch_ctrl2, ID_rs1, ID_rs2, EX_rd, EX_RegWrite, MEM_rd, MEM_RegWrite, MEM_MemRead, WB_rd, WB_RegWrite);
    output [2:0] branch_ctrl1, branch_ctrl2;
    input [4:0] ID_rs1, ID_rs2;
    input [4:0] EX_rd, MEM_rd, WB_rd;
    input EX_RegWrite, MEM_RegWrite, MEM_MemRead, WB_RegWrite;
    
    reg [2:0] branch_ctrl1, branch_ctrl2;
    
    //fowarding branch unit
    always@(*)
    begin
        if( !(EX_rd ^ ID_rs1 ) &&(EX_RegWrite) && (|(EX_rd))  ) 
            branch_ctrl1 = 3'd1;
        else if( !(MEM_rd^ID_rs1) && (MEM_RegWrite) && (|(MEM_rd)) ) begin
            if(MEM_MemRead) begin
                branch_ctrl1 = 3'd2;
            end
            else begin
                branch_ctrl1 = 3'd3;
            end
        end
        else if( !(WB_rd ^ ID_rs1) && (WB_RegWrite) && (|(WB_rd)) ) begin
            branch_ctrl1 = 3'd4;
        end
        else begin
            branch_ctrl1 = 3'd0; 
        end
        
        
        
        if( !(EX_rd ^ ID_rs2) &&(EX_RegWrite) && (|(EX_rd))  ) begin 
            branch_ctrl2 = 3'd1;
        end
        else if( !(MEM_rd ^ ID_rs2) && (MEM_RegWrite) && (|(MEM_rd)) ) begin
            if(MEM_MemRead)
                branch_ctrl2 = 3'd2;
            else
                branch_ctrl2 = 3'd3;
        end
        else if( !(WB_rd ^ ID_rs2) && (WB_RegWrite) && (|(WB_rd)) ) begin
            branch_ctrl2 = 3'd4;
        end
        else begin
            branch_ctrl2 = 3'd0;        
        end
    end
endmodule

module Branch_Cmp(branch_jump, jump_read_data, Branch, branch_ctrl1, branch_ctrl2, ID_rdata1, ID_rdata2, EX_alu_result, MEM_rdata, MEM_alu_result, WB_wdata);
    output branch_jump;
    output [31:0] jump_read_data;
    input [1:0] Branch;
    input [2:0] branch_ctrl1, branch_ctrl2;
    input [31:0] ID_rdata1, ID_rdata2;
    input [31:0] EX_alu_result, MEM_alu_result;
    input [31:0] MEM_rdata, WB_wdata;
    
    reg branch_jump;
    reg [31:0] jump_read_data;
    reg [31:0] branch_data1, branch_data2;
    reg beq, bne;
    always @(*) begin
        case(branch_ctrl1)
            0: 
            begin
                branch_data1   = ID_rdata1;
                jump_read_data = ID_rdata1;
            end
            1:
            begin
                branch_data1   = EX_alu_result;
                jump_read_data = EX_alu_result;
            end
            2:
            begin
                branch_data1   = MEM_rdata;
                jump_read_data = MEM_rdata;
            end            
            3:
            begin
                branch_data1   = MEM_alu_result;
                jump_read_data = MEM_alu_result;
            end
            4:
            begin
                branch_data1   = WB_wdata;
                jump_read_data = WB_wdata;
            end            
            default:
            begin
                branch_data1   = ID_rdata1;
                jump_read_data = ID_rdata1;
            end              
        endcase
        case(branch_ctrl2)
            0: branch_data2 = ID_rdata2;
            1: branch_data2 = EX_alu_result;
            2: branch_data2 = MEM_rdata;
            3: branch_data2 = MEM_alu_result;
            default:branch_data2 = ID_rdata2;
        endcase
        
        beq = !(branch_data1 ^ branch_data2);
        bne = ~beq;
        
        case(Branch)
            0: branch_jump = 0;
            1: branch_jump = beq;
            2: branch_jump = bne;
            default: branch_jump = 0;
        endcase

    end
endmodule

module Predict(clk, rst_n, IF_B, ID_B, predict, stall, BrPred);
    input clk, rst_n;
    input IF_B, ID_B;
    input predict;
    input stall;
    output BrPred;
    
    
    parameter NotTaken1 = 2'b00;  // strong taken
    parameter NotTaken2 = 2'b01;  // strong not taken
    parameter Taken1    = 2'b10;  // strong taken
    parameter Taken2    = 2'b11;  // strong not taken
    
    reg BrPred;
    reg [1:0] pred_state, pred_state_nxt;
    
    always @(*) begin
        if(pred_state[1] && IF_B) begin
            BrPred = 1;
        end
        else begin
            BrPred = 0;
        end
        
        if(!ID_B) pred_state_nxt = pred_state;
        else begin
            case(pred_state)
                NotTaken1: begin
                    if(predict) pred_state_nxt = NotTaken1;
                    else pred_state_nxt = NotTaken2;
                end
                NotTaken2: begin
                    if(predict) pred_state_nxt = NotTaken1;
                    else pred_state_nxt = Taken1;
                end
                Taken1: begin
                    if(predict) pred_state_nxt = Taken1;
                    else pred_state_nxt = Taken2;
                end
                Taken2: begin
                    if(predict) pred_state_nxt = Taken1;
                    else pred_state_nxt = NotTaken1;
                end
            endcase
        end
    end
    
    always @(posedge clk) begin
        if(!rst_n) begin
            pred_state <= NotTaken1;
        end
        else if(stall) begin
            pred_state <= pred_state;
        end
        else begin
            pred_state <= pred_state_nxt;
        end
    end
    
endmodule

module reg_file(clk, rst_n, wen, reg1, reg2, regw, write, read1, read2);
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen;
    input [addr_width-1:0]  reg1, reg2, regw;
    input [BITS-1:0] write;
    output [BITS-1:0] read1, read2;
    
    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [1:word_depth-1];
    integer i;
    
    assign read1 = mem[reg1];
    assign read2 = mem[reg2];
    
    always @(*) begin
        mem[0] = 32'b0;
    
        for(i=1; i<word_depth; i=i+1) begin
            mem_nxt[i] = mem[i];
        end
        if(wen) begin
            mem_nxt[regw] = write;
        end
    end
    
    always @(posedge clk) begin
        if(!rst_n) begin
            for(i=1; i<word_depth; i=i+1) begin
                mem[i] <= 0;
            end
        end
        else begin
            for(i=1; i<word_depth; i=i+1) begin
                mem[i] <= mem_nxt[i];
            end
        end
    end
    
endmodule


module write_buffer(
    clk,
    rst,
    write_buffer_address,
    write_buffer_read,
    write_buffer_write,
    write_buffer_read_data,
    write_buffer_write_data,
    write_buffer_stall,
    mem_addr,
    mem_read,
    mem_write,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    input          clk;
    input          rst;
    
    input  [127:0] mem_rdata;
    input          mem_ready;
    output [127:0] mem_wdata;
    output  [27:0] mem_addr;
    output         mem_write;
    output         mem_read;
    
        
    input  [127:0] write_buffer_write_data;    
    input   [27:0] write_buffer_address;
    input          write_buffer_read;
    input          write_buffer_write;
    output [127:0] write_buffer_read_data;    
    output         write_buffer_stall;


    
    parameter IDLE = 3'd0;
    parameter READ = 3'd1;
    parameter WRITE = 3'd2;
    
    reg  [2:0] state, state_nxt;
    reg  [127:0] write_buffer_read_data_reg, write_buffer_read_data_reg_nxt;
    reg          write_buffer_stall_reg, write_buffer_stall_reg_nxt;
    reg   [27:0] mem_addr_reg, mem_addr_reg_nxt;
    reg          mem_read_reg, mem_read_reg_nxt;
    reg          mem_write_reg, mem_write_reg_nxt;
    reg  [127:0] mem_wdata_reg, mem_wdata_reg_nxt;
    
    always@(*)
    begin
        state_nxt =state;
        case(state)
            IDLE:
            begin
                if(write_buffer_write)
                    state_nxt = WRITE;
                else if (write_buffer_read)
                    state_nxt = READ;
            end
            READ:
            begin
                if(mem_ready)
                    state_nxt = IDLE;
            end
            WRITE:
            begin
                if(mem_ready)
                    state_nxt = IDLE;            
            end
            default:state_nxt =state;
        endcase
    end
    always@(posedge clk or posedge rst)
    begin
        if(rst)
            state<=IDLE;
        else
            state<=state_nxt;
    end

    assign mem_addr = mem_addr_reg;
    assign mem_read = mem_read_reg;
    assign mem_write = mem_write_reg;
    assign mem_wdata = mem_wdata_reg;
    assign write_buffer_stall = write_buffer_stall_reg;
    assign write_buffer_read_data = write_buffer_read_data_reg;    
    
    always @ (*) 
    begin
        case (state)
        IDLE: 
        begin
            if (write_buffer_write) 
            begin
                mem_read_reg_nxt = mem_read_reg;
                mem_write_reg_nxt = 1'b1;
                mem_addr_reg_nxt = write_buffer_address;
                mem_wdata_reg_nxt = write_buffer_write_data;
                write_buffer_read_data_reg_nxt = write_buffer_read_data_reg;
                write_buffer_stall_reg_nxt = 1'b1;             
            end
            else if (write_buffer_read) 
            begin                
                mem_read_reg_nxt = 1'b1;
                mem_write_reg_nxt = mem_write_reg;
                mem_addr_reg_nxt = write_buffer_address;
                mem_wdata_reg_nxt = mem_wdata_reg;
                write_buffer_read_data_reg_nxt = write_buffer_read_data_reg;
                write_buffer_stall_reg_nxt = 1'b1;               
            end
            else begin
                mem_read_reg_nxt = mem_read_reg;
                mem_write_reg_nxt = mem_write_reg;
                mem_addr_reg_nxt = mem_addr_reg;
                mem_wdata_reg_nxt = mem_wdata_reg;
                write_buffer_read_data_reg_nxt = write_buffer_read_data_reg;
                write_buffer_stall_reg_nxt = write_buffer_stall_reg;         
            end
        end
        READ: 
        begin
            if (mem_ready) 
            begin

                mem_read_reg_nxt = 1'b0;
                mem_write_reg_nxt = mem_write_reg;
                mem_addr_reg_nxt = mem_addr_reg;              
                mem_wdata_reg_nxt = mem_wdata_reg;
                write_buffer_stall_reg_nxt = 1'b0;
                write_buffer_read_data_reg_nxt = mem_rdata;                
            end
            else 
            begin
                mem_read_reg_nxt = mem_read_reg;
                mem_write_reg_nxt = mem_write_reg;
                mem_addr_reg_nxt = mem_addr_reg;
                mem_wdata_reg_nxt = mem_wdata_reg;
                write_buffer_read_data_reg_nxt = write_buffer_read_data_reg;
                write_buffer_stall_reg_nxt = write_buffer_stall_reg;                
            end
        end
        WRITE: 
        begin
            if (mem_ready) 
            begin
                mem_read_reg_nxt = mem_read_reg;
                mem_write_reg_nxt = 1'b0;
                mem_addr_reg_nxt = mem_addr_reg;               
                mem_wdata_reg_nxt = mem_wdata_reg;                
                write_buffer_read_data_reg_nxt = write_buffer_read_data_reg;
                write_buffer_stall_reg_nxt = 1'b0;
            end
            else 
            begin
                mem_read_reg_nxt = mem_read_reg;
                mem_write_reg_nxt = mem_write_reg;
                mem_addr_reg_nxt = mem_addr_reg;
                mem_wdata_reg_nxt = mem_wdata_reg;
                write_buffer_read_data_reg_nxt = write_buffer_read_data_reg;
                write_buffer_stall_reg_nxt = write_buffer_stall_reg;
            end
        end        
        default: 
        begin
            mem_read_reg_nxt = mem_read_reg;
            mem_write_reg_nxt = mem_write_reg;
            mem_addr_reg_nxt = mem_addr_reg;
            mem_wdata_reg_nxt = mem_wdata_reg;
            write_buffer_read_data_reg_nxt = write_buffer_read_data_reg;
            write_buffer_stall_reg_nxt = write_buffer_stall_reg;            
        end
        endcase
    end

    always @ (posedge clk or posedge rst) 
    begin
        if (rst) 
        begin
            mem_read_reg <= 1'd0;
            mem_write_reg <= 1'd0;
            mem_wdata_reg <= 128'd0;
            mem_addr_reg <= 28'd0;           
            write_buffer_read_data_reg <= 128'd0;
            write_buffer_stall_reg <= 1'd0;            
        end
        else 
        begin
            mem_read_reg <= mem_read_reg_nxt;
            mem_write_reg <= mem_write_reg_nxt;
            mem_addr_reg <= mem_addr_reg_nxt;
            mem_wdata_reg <= mem_wdata_reg_nxt;
            write_buffer_read_data_reg <= write_buffer_read_data_reg_nxt;
            write_buffer_stall_reg <= write_buffer_stall_reg_nxt;            
        end
    end
endmodule


module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_wdata,
    proc_stall,
    proc_rdata,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);

//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;

    
    parameter IDLE = 3'd0;
    parameter READ = 3'd1;
    parameter DIRTY_READ = 3'd2;
    parameter WRITE_BACK = 3'd3;

    wire [127:0] write_buffer_write_data;
    wire [127:0] write_buffer_read_data;
    wire         write_buffer_stall;
    wire  [27:0] write_buffer_address;
    wire         proc_stall;
    wire  [31:0] proc_rdata;

    reg   [2:0] state, state_nxt;
    
    reg          proc_stall_reg, proc_stall_reg_nxt;
    reg   [31:0] proc_rdata_reg, proc_rdata_reg_nxt;
    
    reg  [154:0] cache_block [0:7];
    reg  [154:0] cache_block_nxt [0:7];
    reg  [127:0] select_block_data;
    reg  [127:0] select_updated_block_data;
    reg   [31:0] select_block_data_change_word;
    reg          hit;

    reg          write_buffer_read;
    reg          write_buffer_write;
    reg          write_buffer_need_reg, write_buffer_need_reg_nxt;
    reg   [27:0] write_buffer_address_reg, write_buffer_address_reg_nxt;
    reg  [127:0] write_buffer_write_data_reg, write_buffer_write_data_reg_nxt;
    reg   [31:0] write_buffer_read_data_word;    
    reg  [127:0] write_buffer_updated_data;
    integer i;

//==== instances ==========================================
    write_buffer no1(
        .clk(clk),
        .rst(proc_reset),
        .write_buffer_address(write_buffer_address),
        .write_buffer_read(write_buffer_read),
        .write_buffer_write(write_buffer_write),
        .write_buffer_read_data(write_buffer_read_data),
        .write_buffer_write_data(write_buffer_write_data),
        .write_buffer_stall(write_buffer_stall),
        .mem_addr(mem_addr),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .mem_rdata(mem_rdata),
        .mem_wdata(mem_wdata),
        .mem_ready(mem_ready)
    );

    always@(*)
    begin
        state_nxt =state;
        case(state)
            IDLE:
            begin
                if (proc_read) 
                begin
                    if (cache_block[proc_addr[4:2]][154]) 
                    begin
                        if (hit) 
                            state_nxt = state;
                        else if(cache_block[proc_addr[4:2]][153])
                        begin
                            if(!write_buffer_stall)
                                state_nxt = WRITE_BACK;
                        end
                        else
                        begin
                             if(!write_buffer_stall)
                                state_nxt = READ;                           
                        end
                    end
                    else
                    begin
                        if (!write_buffer_stall)
                        begin
                            state_nxt = READ;
                        end
                    end
                end
                else if (proc_write)
                begin
                    if (cache_block[proc_addr[4:2]][154] & hit)
                    begin
                    end
                    else if (cache_block[proc_addr[4:2]][154] & cache_block[proc_addr[4:2]][153])
                    begin
                        if (~write_buffer_stall)
                            state_nxt = DIRTY_READ;
                    end
                    else
                    begin
                        if (~write_buffer_stall) 
                            state_nxt = DIRTY_READ;
                    end
                end

            end
            READ:
            begin
                if (~write_buffer_stall)
                    state_nxt = IDLE;
            end
            DIRTY_READ:
            begin
                if (~write_buffer_stall)
                    state_nxt = IDLE;            
            end
            WRITE_BACK:
            begin
                if (~write_buffer_stall)
                    state_nxt = IDLE;            
            end            
            default:state_nxt =state;
        endcase
    end
    
    always@(posedge clk or posedge proc_reset)
    begin
        if(proc_reset)
            state<=IDLE;
        else
            state<=state_nxt;
    end
    
    assign proc_stall = proc_stall_reg_nxt;
    assign proc_rdata = proc_rdata_reg_nxt;
    assign write_buffer_address = write_buffer_address_reg_nxt;
    assign write_buffer_write_data = write_buffer_write_data_reg_nxt;

    always @ (*) begin
        select_block_data = cache_block[proc_addr[4:2]][127:0];
        hit = !(proc_addr[29:5] ^ cache_block[proc_addr[4:2]][152:128]);
        case (proc_addr[1:0])
            2'd0: begin
                select_block_data_change_word = select_block_data[31:0];
                select_updated_block_data = {select_block_data[127:32], proc_wdata};
                write_buffer_read_data_word = write_buffer_read_data[31:0];
                write_buffer_updated_data = {write_buffer_read_data[127:32], proc_wdata};
            end
            2'd1: begin
                select_block_data_change_word = select_block_data[63:32];
                select_updated_block_data = {select_block_data[127:64], proc_wdata, select_block_data[31:0]};
                write_buffer_read_data_word = write_buffer_read_data[63:32];
                write_buffer_updated_data = {write_buffer_read_data[127:64], proc_wdata, write_buffer_read_data[31:0]};
            end
            2'd2: begin
                select_block_data_change_word = select_block_data[95:64];
                select_updated_block_data = {select_block_data[127:96], proc_wdata, select_block_data[63:0]};
                write_buffer_read_data_word = write_buffer_read_data[95:64];
                write_buffer_updated_data = {write_buffer_read_data[127:96], proc_wdata, write_buffer_read_data[63:0]};
            end
            2'd3: begin
                select_block_data_change_word = select_block_data[127:96];
                select_updated_block_data = {proc_wdata, select_block_data[95:0]};
                write_buffer_read_data_word = write_buffer_read_data[127:96];
                write_buffer_updated_data = {proc_wdata, write_buffer_read_data[95:0]};
            end
        endcase
    end

    always @ (*) 
    begin
        case (state)
        IDLE: 
        begin
            if (proc_read) 
            begin
                if (cache_block[proc_addr[4:2]][154]) 
                begin
                    if (hit) 
                    begin
                        proc_rdata_reg_nxt = select_block_data_change_word;
                        proc_stall_reg_nxt = 1'b0;
                        for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                        write_buffer_address_reg_nxt = proc_addr[29:2];
                        write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                        write_buffer_need_reg_nxt = write_buffer_need_reg;
                        write_buffer_read = 1'b0;
                        write_buffer_write = 1'b0;
                    end
                    else if (cache_block[proc_addr[4:2]][153]) 
                    begin
                        if (!write_buffer_stall) 
                        begin
                            write_buffer_read = 1'b1;
                            write_buffer_write = 1'b0;
                            write_buffer_write_data_reg_nxt = select_block_data;
                            proc_stall_reg_nxt = 1'b1;
                            for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                            proc_rdata_reg_nxt = proc_rdata_reg;
                            write_buffer_address_reg_nxt = proc_addr[29:2];
                            write_buffer_need_reg_nxt = write_buffer_need_reg;
                        end
                        else 
                        begin
                            for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                            proc_stall_reg_nxt = 1'b1;
                            proc_rdata_reg_nxt = proc_rdata_reg;
                            write_buffer_address_reg_nxt = proc_addr[29:2];
                            write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                            write_buffer_need_reg_nxt = write_buffer_need_reg;
                            write_buffer_read = 1'b0;
                            write_buffer_write = 1'b0;
                        end
                    end
                    else 
                    begin
                        if (~write_buffer_stall) 
                        begin
                            write_buffer_read = 1'b1;
                            write_buffer_write = 1'b0;
                            proc_stall_reg_nxt = 1'b1;
                            for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                            proc_rdata_reg_nxt = proc_rdata_reg;
                            write_buffer_address_reg_nxt = proc_addr[29:2];
                            write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                            write_buffer_need_reg_nxt = write_buffer_need_reg;
                        end
                        else 
                        begin
                            for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                            proc_stall_reg_nxt = 1'b1;
                            proc_rdata_reg_nxt = proc_rdata_reg;
                            write_buffer_address_reg_nxt = proc_addr[29:2];
                            write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                            write_buffer_need_reg_nxt = write_buffer_need_reg;
                            write_buffer_read = 1'b0;
                            write_buffer_write = 1'b0;
                        end
                    end
                end
                else 
                begin
                    if (!write_buffer_stall) 
                    begin
                        write_buffer_read = 1'b1;
                        write_buffer_write = 1'b0;
                        proc_stall_reg_nxt = 1'b1;                       
                        for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                        proc_rdata_reg_nxt = proc_rdata_reg;
                        write_buffer_address_reg_nxt = proc_addr[29:2];
                        write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                        write_buffer_need_reg_nxt = write_buffer_need_reg;
                    end
                    else 
                    begin
                        for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                        proc_stall_reg_nxt = 1'b1;
                        proc_rdata_reg_nxt = proc_rdata_reg;
                        write_buffer_address_reg_nxt = proc_addr[29:2];
                        write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                        write_buffer_need_reg_nxt = write_buffer_need_reg;
                        write_buffer_read = 1'b0;
                        write_buffer_write = 1'b0;
                    end
                end
            end
            else if (proc_write)
            begin
                if (cache_block[proc_addr[4:2]][154] & hit) 
                begin
                    for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                    proc_rdata_reg_nxt = proc_rdata_reg;
                    write_buffer_address_reg_nxt = proc_addr[29:2];
                    write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                    write_buffer_need_reg_nxt = write_buffer_need_reg;
                    write_buffer_read = 1'b0;
                    write_buffer_write = 1'b0;
                    cache_block_nxt[proc_addr[4:2]] = {1'b1, 1'b1, proc_addr[29:5], select_updated_block_data};
                    proc_stall_reg_nxt = 1'b0;
                end
                else if (cache_block[proc_addr[4:2]][154] & cache_block[proc_addr[4:2]][153]) 
                begin
                    if (~write_buffer_stall) 
                    begin
                        write_buffer_write_data_reg_nxt = select_block_data;
                        write_buffer_read = 1'b1;
                        write_buffer_write = 1'b0;
                        write_buffer_need_reg_nxt = 1'b1;
                        proc_stall_reg_nxt = 1'b1;
                        for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                        proc_rdata_reg_nxt = proc_rdata_reg;
                        write_buffer_address_reg_nxt = proc_addr[29:2];
                    end
                    else 
                    begin
                        for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                        proc_stall_reg_nxt = 1'b1;
                        proc_rdata_reg_nxt = proc_rdata_reg;
                        write_buffer_address_reg_nxt = proc_addr[29:2];
                        write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                        write_buffer_need_reg_nxt = write_buffer_need_reg;
                        write_buffer_read = 1'b0;
                        write_buffer_write = 1'b0;
                    end
                end
                else begin
                    if (~write_buffer_stall) 
                    begin
                        write_buffer_read = 1'b1;
                        write_buffer_write = 1'b0;
                        proc_stall_reg_nxt = 1'b1;
                        for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                        proc_rdata_reg_nxt = proc_rdata_reg;
                        write_buffer_address_reg_nxt = proc_addr[29:2];
                        write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                        write_buffer_need_reg_nxt = write_buffer_need_reg;
                    end
                    else 
                    begin
                        for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                        proc_stall_reg_nxt = 1'b1;
                        proc_rdata_reg_nxt = proc_rdata_reg;
                        write_buffer_address_reg_nxt = proc_addr[29:2];
                        write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                        write_buffer_need_reg_nxt = write_buffer_need_reg;
                        write_buffer_read = 1'b0;
                        write_buffer_write = 1'b0;
                    end
                end
            end
            else 
            begin
                for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                proc_stall_reg_nxt = proc_stall_reg;
                proc_rdata_reg_nxt = proc_rdata_reg;
                write_buffer_address_reg_nxt = proc_addr[29:2];
                write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                write_buffer_need_reg_nxt = write_buffer_need_reg;
                write_buffer_read = 1'b0;
                write_buffer_write = 1'b0;
            end
        end
        READ: 
        begin
            if (!write_buffer_stall) 
            begin
                for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                write_buffer_address_reg_nxt = proc_addr[29:2];
                write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                write_buffer_need_reg_nxt = write_buffer_need_reg;
                write_buffer_read = 1'b0;
                write_buffer_write = 1'b0;
                proc_rdata_reg_nxt = write_buffer_read_data_word;
                proc_stall_reg_nxt = 1'b0;
                cache_block_nxt[proc_addr[4:2]] = {1'b1, 1'b0, proc_addr[29:5], write_buffer_read_data};
            end
            else 
            begin
                for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                proc_stall_reg_nxt = proc_stall_reg;
                proc_rdata_reg_nxt = proc_rdata_reg;
                write_buffer_address_reg_nxt = proc_addr[29:2];
                write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                write_buffer_need_reg_nxt = write_buffer_need_reg;
                write_buffer_read = 1'b0;
                write_buffer_write = 1'b0;
            end
        end
        DIRTY_READ: 
        begin
            if (~write_buffer_stall) 
            begin
                if (~write_buffer_need_reg) 
                begin
                    for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                    proc_rdata_reg_nxt = proc_rdata_reg;
                    write_buffer_address_reg_nxt = proc_addr[29:2];
                    write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                    write_buffer_need_reg_nxt = write_buffer_need_reg;
                    proc_stall_reg_nxt = 1'b0;
                    cache_block_nxt[proc_addr[4:2]] = {1'b1, 1'b1, proc_addr[29:5], write_buffer_updated_data};
                    write_buffer_read = 1'b0;
                    write_buffer_write = 1'b0;
                end
                else 
                begin
                    for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                    proc_rdata_reg_nxt = proc_rdata_reg;
                    write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                    proc_stall_reg_nxt = 1'b0;
                    cache_block_nxt[proc_addr[4:2]] = {1'b1, 1'b1, proc_addr[29:5], write_buffer_updated_data};

                    write_buffer_write = 1'b1;
                    write_buffer_read = 1'b0;
                    write_buffer_address_reg_nxt = {cache_block[proc_addr[4:2]][152:128], proc_addr[4:2]};
                    write_buffer_need_reg_nxt = 1'b0;
                end
            end
            else 
            begin
                for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                proc_stall_reg_nxt = proc_stall_reg;
                proc_rdata_reg_nxt = proc_rdata_reg;
                write_buffer_address_reg_nxt = proc_addr[29:2];
                write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                write_buffer_need_reg_nxt = write_buffer_need_reg;
                write_buffer_read = 1'b0;
                write_buffer_write = 1'b0;
            end
        end
        WRITE_BACK: 
        begin
            if (~write_buffer_stall) 
            begin
                for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                write_buffer_need_reg_nxt = write_buffer_need_reg;

                write_buffer_read = 1'b0;
                write_buffer_write = 1'b1;
                write_buffer_address_reg_nxt = {cache_block[proc_addr[4:2]][152:128], proc_addr[4:2]};
                proc_rdata_reg_nxt = write_buffer_read_data_word;
                proc_stall_reg_nxt = 1'b0;
                cache_block_nxt[proc_addr[4:2]] = {1'b1, 1'b0, proc_addr[29:5], write_buffer_read_data};
            end
            else 
            begin
                for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
                proc_stall_reg_nxt = proc_stall_reg;
                proc_rdata_reg_nxt = proc_rdata_reg;
                write_buffer_address_reg_nxt = proc_addr[29:2];
                write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
                write_buffer_need_reg_nxt = write_buffer_need_reg;
                write_buffer_read = 1'b0;
                write_buffer_write = 1'b0;
            end
        end
        default:
        begin
            for (i=0; i<8; i=i+1) cache_block_nxt[i] = cache_block[i];
            proc_stall_reg_nxt = proc_stall_reg;
            proc_rdata_reg_nxt = proc_rdata_reg;
            write_buffer_address_reg_nxt = proc_addr[29:2];
            write_buffer_write_data_reg_nxt = write_buffer_write_data_reg;
            write_buffer_need_reg_nxt = write_buffer_need_reg;
            write_buffer_read = 1'b0;
            write_buffer_write = 1'b0;
        end 
        endcase
    end

//==== sequential circuit =================================
    always @ (posedge clk or posedge proc_reset) begin
        if(proc_reset) begin
            for (i=0; i<8; i=i+1) cache_block[i] <= 155'b0;
            proc_stall_reg <= 1'b0;
            proc_rdata_reg <= 32'b0;
            write_buffer_address_reg <= 28'b0;
            write_buffer_write_data_reg <= 128'b0;
            write_buffer_need_reg <= 1'b0;
        end
        else begin
            for (i=0; i<8; i=i+1) cache_block[i] <= cache_block_nxt[i];
            proc_stall_reg <= proc_stall_reg_nxt;
            proc_rdata_reg <= proc_rdata_reg_nxt;
            write_buffer_address_reg <= write_buffer_address_reg_nxt;
            write_buffer_write_data_reg <= write_buffer_write_data_reg_nxt;
            write_buffer_need_reg <= write_buffer_need_reg_nxt;
        end
    end
endmodule