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
				DCACHE_wen,
        PC
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
output  [31:0]  PC;
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
		.DCACHE_rdata   (DCACHE_rdata)  ,
    .PC             (PC)
	);
	

	Dcache D_cache(
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

	Icache I_cache(
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
    DCACHE_rdata,
    PC
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
    output [31:0] PC;
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
    reg    [31:0] IF_PC42;
    reg           IF_flush;
    reg           IF_PCSrc;
    
    reg    [31:0] ID_PC, ID_PC_nxt;
    reg    [31:0] ID_instruction;
    wire   [31:0] ID_instruction_nxt;
    reg    [31:0] ID_original_inst;
    wire   [31:0] ID_original_inst_wire;
    wire   [31:0] ID_instruction_wire;
    wire   [6:0]  ID_opcode;
    wire   [6:0]  ID_funct7;
    wire   [2:0]  ID_funct3;
    reg           ID_flush;
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
    wire          ID_hazard;
    reg    [31:0] ID_jump_address;
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
    reg           EX_branch_jump;
    reg    [31:0] EX_jump_address;
    
    wire          EX_zero;
    
    
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
    reg           MEM_branch_jump, MEM_branch_jump_nxt;
    reg           MEM_jump, MEM_jump_nxt;
    
    reg    [4:0]  WB_rd, WB_rd_nxt;
    wire   [4:0]  WB_rd_wire;
    wire          WB_RegWrite_wire;
    reg           WB_RegWrite, WB_RegWrite_nxt;
    reg           WB_MemtoReg, WB_MemtoReg_nxt;
    reg    [31:0] WB_alu_result, WB_alu_result_nxt;
    reg    [31:0] WB_DCACHE_rdata, WB_DCACHE_rdata_nxt;
    wire   [31:0] WB_regfile_wdata;
    
    reg           IF_compressed;
    reg           ID_compressed, ID_compressed_nxt;
    reg           EX_compressed, EX_compressed_nxt;
    reg    [15:0] low_data, low_data_nxt;
    reg    [15:0] high_data, high_data_nxt;
    reg           inst_look_forward, inst_look_forward_nxt;
    
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
    
    ALU ALU_Unit(.data1(EX_alu_data1), .data2(EX_alu_data2), .ALU_operation(EX_alu_operation), .result(EX_alu_result), .zero(EX_zero));
    
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
    
    Decompressor Decompressor_Unit(
        .inst(ID_original_inst_wire),
        .decompressed_inst(ID_instruction_nxt)
    );
    // continuous assignment---------------------------------------------
    assign PC = IF_PC;
    assign ID_original_inst_wire = ID_original_inst;
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
    assign EX_alu_data2 = (EX_ALUSrc2[1])? ((EX_compressed)? 32'd2 : 32'd4) : ((EX_ALUSrc2[0])? EX_immediate : EX_forwardB_data);  //2'b10 -> 32'd4, 2'b01 -> Ex_immedate, 2'b00 -> Ex_forwardB_data
    assign MEM_rd_wire = MEM_rd;
    assign WB_rd_wire = WB_rd;
    assign MEM_RegWrite_wire = MEM_RegWrite;
    assign EX_RegWrite_wire = EX_RegWrite;
    assign MEM_MemRead_wire = MEM_MemRead;
    assign MEM_DCACHE_rdata = {DCACHE_rdata[7:0], DCACHE_rdata[15:8], DCACHE_rdata[23:16], DCACHE_rdata[31:24]};
    assign MEM_alu_result_wire = MEM_alu_result;
    assign WB_regfile_wdata = (WB_MemtoReg)?  WB_DCACHE_rdata : WB_alu_result;
    
    // PC DFF-----------------------------------------------------------
    always @(posedge clk) begin
       if(!rst_n) begin
           IF_PC <= 32'b0;
           low_data <= 16'b0;
           high_data <= 16'b0;
           inst_look_forward <= 1'b0;
       end
       else if(IF_PC_stall) begin
           IF_PC <= IF_PC;
           low_data <= low_data;
           high_data <= high_data;
           inst_look_forward <= inst_look_forward;
       end
       else begin
           IF_PC <= IF_PC_nxt;
           low_data <= low_data_nxt;
           high_data <= high_data_nxt;
           inst_look_forward <= inst_look_forward_nxt;
       end
    end
    
    // IF stage --------------------------------------------------------------
    assign IF_PC_nxt = (inst_look_forward && (MEM_branch_jump || MEM_jump))? IF_PC : (
                            (IF_PCSrc)? EX_jump_address : IF_PC42
                        );
    always @(*) begin
        IF_PCSrc = (EX_branch_jump || EX_jump);
        IF_PC42 = (IF_compressed)? IF_PC + 32'd2 : IF_PC + 32'd4;
        
        low_data_nxt = low_data;
        high_data_nxt = high_data;
        inst_look_forward_nxt = 0;
        
        
        if(EX_jump || EX_branch_jump) begin
            if(IF_PC_nxt[1]) inst_look_forward_nxt = 1;
            else inst_look_forward_nxt = 0;
        end
        else begin
            inst_look_forward_nxt = 0;
        end
        
 
        if(IF_PC[1]) ICACHE_addr = (inst_look_forward)?  IF_PC[31:2] : IF_PC[31:2] + 1; // IF_PC-2 or IF_PC+2;
        else ICACHE_addr = IF_PC[31:2];
        
        if(IF_PC[1]) begin
            if(inst_look_forward) begin
                low_data_nxt = ICACHE_rdata[15:0];
                high_data_nxt = 8'b00_00_01_00;          // nop for this cycle
            end
            else begin
                low_data_nxt = ICACHE_rdata[15:0]; 
                high_data_nxt = low_data;
                
            end
        end
        else begin
            low_data_nxt  = ICACHE_rdata[15:0];
            high_data_nxt = ICACHE_rdata[31:16];  
         
        end
        
        if(inst_look_forward) begin
            if(low_data_nxt[9:8] == 2'b11) IF_compressed = 0;
            else IF_compressed = 1;
        end
        else begin
            if(high_data_nxt[9:8] == 2'b11) IF_compressed = 0;
            else IF_compressed = 1;
        end
            
        if(IF_compressed) begin
            if(inst_look_forward) ID_original_inst = {16'b0, 16'b00_00_00_01};
            else ID_original_inst = {16'b0, high_data_nxt[7:0], high_data_nxt[15:8]};
        end
        else begin  // 32b instruction
            if(IF_PC[1]) begin  // at PC = multiple of 2
                if(inst_look_forward) ID_original_inst = {16'b0, 16'b00_00_00_01};
                else ID_original_inst = {ICACHE_rdata[23:16], ICACHE_rdata[31:24], high_data_nxt[7:0], high_data_nxt[15:8]};
            end
            else begin  // at PC = multiple of 4
                ID_original_inst = {low_data_nxt[7:0], low_data_nxt[15:8], high_data_nxt[7:0], high_data_nxt[15:8]};
            end
        end
        
        ID_compressed_nxt = IF_compressed;
        ICACHE_ren = 1'b1;
        ICACHE_wen = 1'b0;
        ICACHE_wdata = 32'b0; 
        IF_flush = IF_PCSrc;
        
        ID_PC_nxt = IF_PC;
                
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
            ID_instruction <= 32'b0;
            ID_compressed <= 0;
        end
        else if(IF_ID_stall) begin 
            ID_PC<=ID_PC;
            ID_instruction<=ID_instruction;
            ID_compressed <= ID_compressed;
        end
        else begin
            if(IF_flush) ID_instruction <= {25'd0,7'b0010011};
            else ID_instruction <= ID_instruction_nxt;
            
            ID_PC <= ID_PC_nxt;
            ID_compressed <= ID_compressed_nxt;
        end
    end
    
    // ID stage ---------------------------------------------------------------
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
    always @(*) begin
        ID_flush = IF_PCSrc;
         
        
        if(ID_hazard || ID_flush) begin
            EX_PC_nxt = ID_PC;
            EX_funct7_nxt = 0;
            EX_funct3_nxt = 0;
            EX_rs1_nxt = 0;
            EX_rs2_nxt = 0;
            EX_rd_nxt  = 0;
            EX_compressed_nxt = 0;
            
            EX_wdata_nxt  = 0;
            EX_immediate_nxt = 0;
            EX_RegWrite_nxt = 0;
            EX_ALUSrc1_nxt = 0;
            EX_ALUSrc2_nxt = 0;
            EX_ALUOP_nxt = 0;
            EX_MemWrite_nxt = 0;
            EX_MemtoReg_nxt = 0;
            EX_MemRead_nxt = 0;
            EX_Branch_nxt = 0;
            EX_jump_nxt = 0;
            EX_Jalr_nxt = 0;
        end
        else begin
            EX_PC_nxt = ID_PC;
            EX_funct7_nxt = ID_funct7;
            EX_funct3_nxt = ID_funct3;
            EX_rs1_nxt = ID_rs1;
            EX_rs2_nxt = ID_rs2;
            EX_rd_nxt  = ID_rd;
            EX_compressed_nxt = ID_compressed;
            
            
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
            EX_compressed <= 0;
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
            EX_compressed <= EX_compressed;
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
            EX_compressed <= EX_compressed_nxt;
        end
    end
    
    // EX stage
    always @(*) begin
        EX_jump_address = (EX_Jalr)? (EX_alu_data1 + EX_immediate) : (EX_PC + EX_immediate); 
        
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
        
        if(EX_Branch == 2'b01 && EX_zero) EX_branch_jump = 1;
        else if(EX_Branch == 2'b10 && !EX_zero) EX_branch_jump = 1;
        else EX_branch_jump = 0; 
        
        
        
        MEM_rd_nxt = EX_rd;
        MEM_RegWrite_nxt = EX_RegWrite;
        MEM_MemWrite_nxt = EX_MemWrite;
        MEM_MemtoReg_nxt = EX_MemtoReg;
        MEM_MemRead_nxt  = EX_MemRead;
        MEM_alu_result_nxt = EX_alu_result;
        MEM_DCACHE_wdata_nxt = EX_forwardB_data;
        MEM_branch_jump_nxt = EX_branch_jump;
        MEM_jump_nxt = EX_jump;
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
            MEM_branch_jump <= 0;
            MEM_jump <= 0;
        end
        else if(EX_MEM_stall) begin
            MEM_rd <= MEM_rd;
            MEM_RegWrite <= MEM_RegWrite;
            MEM_MemWrite <= MEM_MemWrite;
            MEM_MemtoReg <= MEM_MemtoReg;
            MEM_MemRead  <=  MEM_MemRead;
            MEM_alu_result <= MEM_alu_result;
            MEM_DCACHE_wdata <= MEM_DCACHE_wdata;
            MEM_branch_jump <= MEM_branch_jump;
            MEM_jump <= MEM_jump;
        end
        else begin
            MEM_rd <= MEM_rd_nxt;
            MEM_RegWrite <= MEM_RegWrite_nxt;
            MEM_MemWrite <= MEM_MemWrite_nxt;
            MEM_MemtoReg <= MEM_MemtoReg_nxt;
            MEM_MemRead  <=  MEM_MemRead_nxt;
            MEM_alu_result <= MEM_alu_result_nxt;
            MEM_DCACHE_wdata <= MEM_DCACHE_wdata_nxt;
            MEM_branch_jump <= MEM_branch_jump_nxt;
            MEM_jump <= MEM_jump_nxt;
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
                ALUSrc2 = 2'b10;  //PC + 4;
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
                ALUSrc1 = 0;
                ALUSrc2 = 2'b10;  //PC + 4
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

module ALU(data1, data2, ALU_operation, result, zero);
    input  signed [31:0] data1, data2;
    input  [3:0]  ALU_operation;
    output signed [31:0] result;
    output zero;
    
    reg    signed [31:0] result, subtraction;
    reg    zero;
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
        
        zero = ~(data1 ^ data2);
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
        
        if((MEM_RegWrite) && (!(MEM_rd ^EX_rs1)) && (|(MEM_rd)) )
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
module Decompressor(inst, decompressed_inst);
    input [31:0] inst;
    output [31:0] decompressed_inst;
    reg [31:0] decompressed_inst;
    wire [4:0]  mapped_reg1, mapped_reg2;
    
    wire [2:0] funct3;
    wire [1:0] op;
    wire compressed;

    
    assign compressed = (inst[1:0] == 2'b11)? 1'b0:1'b1;
    assign funct3 = inst[15:13];
    assign op = inst[1:0];
    assign mapped_reg1 = {2'b01, inst[9:7]};
    assign mapped_reg2  = {2'b01, inst[4:2]};
    
    always @(*) begin
        decompressed_inst = 32'b0;
        if(compressed) begin
            case(funct3)
                3'b000: begin
                    case(op)
                        2'b01: begin
                            if(inst[11:7] == 0) begin  // C.NOP
                                decompressed_inst = 32'b000000000000_00000_000_00000_0010011;
                            end
                            else begin  //C.ADDI
                                decompressed_inst = { {7{inst[12]}}, inst[6:2], inst[11:7], 3'b000, inst[11:7], 7'b0010011};
                            end
                        end
                        2'b10: begin  // C.SLLI
                            decompressed_inst = {7'b0, inst[12], inst[6:2], inst[11:7], 3'b001, inst[11:7], 7'b0010011};
                        end
                        default: decompressed_inst = 32'b0;
                    endcase
                end
                3'b001: begin  // C.JAL  // {inst[12]} {inst[8], inst[10:9], inst[6], inst[7], inst[2], inst[11], inst[5:3]} inst[12], 8{inst[12]}
                    decompressed_inst = {inst[12], {inst[8], inst[10:9], inst[6], inst[7], inst[2], inst[11], inst[5:3]}, inst[12], {8{inst[12]}}, 5'b00001, 7'b1101111};
                end
                3'b010: begin  // C.LW
                    decompressed_inst = {5'b0, {inst[5], inst[12:10], inst[6], 2'b00}, mapped_reg1, 3'b010, mapped_reg2, 7'b0000011};
                end
                3'b100: begin
                    case(op)
                        2'b01: begin
                            case(inst[11:10])
                                2'b00: begin // C.SRLI
                                    decompressed_inst = {7'b0000000,  inst[6:2], mapped_reg1, 3'b101, mapped_reg1, 7'b0010011};
                                end
                                2'b01: begin // C.SRAI
                                    decompressed_inst = {7'b0100000,  inst[6:2], mapped_reg1, 3'b101, mapped_reg1, 7'b0010011};
                                end
                                2'b10: begin // C.ANDI
                                    decompressed_inst = {{{7{inst[5]}}, inst[6:2]}, mapped_reg1, 3'b111, mapped_reg1, 7'b0010011};
                                end
                                default: decompressed_inst = 32'b0;
                            endcase
                        end
                        2'b10: begin
                            if(inst[12]) begin
                                if(inst[6:2] == 0) begin  // C.JALR
                                    decompressed_inst = {12'b0, inst[11:7], 3'b000, 5'b00001, 7'b1100111};
                                end
                                else begin  // C.ADD
                                    decompressed_inst = {7'b0000000, inst[6:2], inst[11:7], 3'b000, inst[11:7], 7'b0110011};
                                end
                            end
                            else begin
                                if(inst[6:2] == 0) begin  // C.JR
                                    decompressed_inst = {12'b0, inst[11:7], 3'b000, 5'b00000, 7'b1100111};
                                end
                                else begin  // C.MV
                                    decompressed_inst = {7'b0000000, inst[6:2], 5'b00000, 3'b000, inst[11:7], 7'b0110011};
                                end
                            end
                        end
                    endcase
                end
                3'b101: begin  // C.J
                    decompressed_inst = {inst[12], {inst[8], inst[10:9], inst[6], inst[7], inst[2], inst[11], inst[5:3]}, inst[12], {8{inst[12]}}, 5'b00000, 7'b1101111};
                end
                3'b110: begin
                    if(op == 2'b00) begin  // C.SW
                        decompressed_inst = {{5'b0, inst[5], inst[12]}, mapped_reg2, mapped_reg1, 3'b010, {inst[11:10], inst[6], 2'b00}, 7'b0100011};
                    end
                    else if(op == 2'b01) begin  // C.BEQZ
                        decompressed_inst = {{{4{inst[12]}}, inst[6:5], inst[2]}, 5'b0, mapped_reg1, 3'b000, {inst[11:10], inst[4:3], inst[12]}, 7'b1100011};
                    end
                    else decompressed_inst = 32'b0;

                end
                3'b111: begin  // C.BNEZ
                    decompressed_inst = {{{4{inst[12]}}, inst[6:5], inst[2]}, 5'b0, mapped_reg1, 3'b001, {inst[11:10], inst[4:3], inst[12]}, 7'b1100011};
                end
                default: decompressed_inst = 32'b0;
            endcase
        end
        else begin
            decompressed_inst = inst;
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

    reg          write_buffer_stall_reg, write_buffer_stall_reg_nxt;
    reg   [27:0] mem_addr_reg, mem_addr_reg_nxt;
    reg          mem_read_reg, mem_read_reg_nxt;
    reg          mem_write_reg, mem_write_reg_nxt;
    reg  [127:0] write_buffer_reg, write_buffer_nxt;
    
    reg  mem_ready_reg ;
    // wire mem_ready_nxt, ;
    reg [127:0]  mem_rdata_reg; 
    // wire [127:0] mem_rdata_nxt;
    // assign mem_ready_nxt = mem_ready_reg;
    // assign mem_rdata_nxt = mem_rdata_reg;

    always @(posedge clk or posedge rst) begin
        if(rst)begin
            mem_rdata_reg <= 128'd0;
            mem_ready_reg <= 1'd0;
        end else begin
            mem_rdata_reg <= mem_rdata;
            mem_ready_reg <= mem_ready;
        end
    end

    // always
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
                if(mem_ready_reg)
                    state_nxt = IDLE;
            end
            WRITE:
            begin
                if(mem_ready_reg)
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
    assign mem_read = (mem_ready)? 1'd0 : mem_read_reg;
    assign mem_write =(mem_ready)? 1'd0 : mem_write_reg;
    assign mem_wdata = write_buffer_reg;
    assign write_buffer_stall = write_buffer_stall_reg;  

    assign write_buffer_read_data = write_buffer_reg; 
    
    always @ (*) 
    begin
        case (state)
        IDLE: 
        begin
            if (write_buffer_write) 
            begin
                mem_read_reg_nxt = mem_read_reg;
                mem_write_reg_nxt = 1'b1;
                mem_addr_reg_nxt = write_buffer_address; //input ¾×flip flop
                write_buffer_nxt = write_buffer_write_data; // input ¾×flip flop

                write_buffer_stall_reg_nxt = 1'b1;             
            end
            else if (write_buffer_read) 
            begin    // read data from memory            
                mem_read_reg_nxt = 1'b1;
                mem_write_reg_nxt = mem_write_reg;
                mem_addr_reg_nxt = write_buffer_address; //input
                write_buffer_nxt = write_buffer_reg;

                write_buffer_stall_reg_nxt = 1'b1;               
            end
            else begin
                mem_read_reg_nxt = mem_read_reg;
                mem_write_reg_nxt = mem_write_reg;
                mem_addr_reg_nxt = mem_addr_reg;
                write_buffer_nxt = write_buffer_reg;
                // write_buffer_read_data_reg_nxt = write_buffer_read_data_reg;
                write_buffer_stall_reg_nxt = write_buffer_stall_reg;         
            end
        end
        READ: 
        begin
            if (mem_ready_reg) 
            begin

                mem_read_reg_nxt = 1'b0;
                mem_write_reg_nxt = mem_write_reg;
                mem_addr_reg_nxt = mem_addr_reg;              
                write_buffer_nxt = mem_rdata_reg;

                write_buffer_stall_reg_nxt = 1'b0;
                          
            end
            else 
            begin
                mem_read_reg_nxt = mem_read_reg;
                mem_write_reg_nxt = mem_write_reg;
                mem_addr_reg_nxt = mem_addr_reg;
                write_buffer_nxt = write_buffer_reg;

                write_buffer_stall_reg_nxt = write_buffer_stall_reg;                
            end
        end
        WRITE: 
        begin
            if (mem_ready_reg) 
            begin
                mem_read_reg_nxt = mem_read_reg;
                mem_write_reg_nxt = 1'b0;
                mem_addr_reg_nxt = mem_addr_reg;               
                write_buffer_nxt = write_buffer_reg;                

                write_buffer_stall_reg_nxt = 1'b0;
            end
            else 
            begin
                mem_read_reg_nxt = mem_read_reg;
                mem_write_reg_nxt = mem_write_reg;
                mem_addr_reg_nxt = mem_addr_reg;
                write_buffer_nxt = write_buffer_reg;

                write_buffer_stall_reg_nxt = write_buffer_stall_reg;
            end
        end        
        default: 
        begin
            mem_read_reg_nxt = mem_read_reg;
            mem_write_reg_nxt = mem_write_reg;
            mem_addr_reg_nxt = mem_addr_reg;
            write_buffer_nxt = write_buffer_reg;

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
            write_buffer_reg <= 128'd0;
            mem_addr_reg <= 28'd0;           

            write_buffer_stall_reg <= 1'd0;            
        end
        else 
        begin
            mem_read_reg <= mem_read_reg_nxt;
            mem_write_reg <= mem_write_reg_nxt;
            mem_addr_reg <= mem_addr_reg_nxt;
            write_buffer_reg <= write_buffer_nxt;

            write_buffer_stall_reg <= write_buffer_stall_reg_nxt;            
        end
    end
endmodule


module Dcache(
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
    parameter WRITE_AllOCATE = 3'd2;
    parameter WRITE_BACK = 3'd3;

    wire [127:0] write_buffer_write_data;
    wire [127:0] write_buffer_read_data;
    wire         write_buffer_stall;
    wire  [27:0] write_buffer_address;
    wire         proc_stall;
    wire  [31:0] proc_rdata;
    wire valid, dirty;
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

    reg   [27:0] write_buffer_address_reg, write_buffer_address_reg_nxt;
    reg  [127:0] write_buffer_write_data_reg, write_buffer_write_data_reg_nxt;
    reg   [31:0] write_buffer_read_data_word;    
    reg  [127:0] write_buffer_updated_data;
    reg  write_buffer_need_reg_nxt ,write_buffer_need_reg;
    integer i;

//==== instances ==========================================
    write_buffer U1(
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


    assign valid = cache_block[proc_addr[4:2]][154];
    assign dirty = cache_block[proc_addr[4:2]][153];
    always@(*)
    begin
        state_nxt =state;
        case(state)
            IDLE:
            begin
                if (proc_read) 
                begin


                    if (valid & hit) 
                        state_nxt = state;
                    else if(valid &dirty)
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
                else if (proc_write)
                begin

                    if (valid & hit)
                    begin
                        state_nxt = state;
                    end
                    else 
                    begin
                        if (~write_buffer_stall)
                            state_nxt = WRITE_AllOCATE;
                    end





                end

            end
            READ:
            begin
                if (~write_buffer_stall)
                    state_nxt = IDLE;
            end
            WRITE_AllOCATE:
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
                if (valid) 
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
                    else if (dirty) 
                    begin
                        if (!write_buffer_stall) 
                        begin // Write back
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
                if (valid & hit) 
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
                else if (valid & dirty) 
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
        WRITE_AllOCATE: 
        begin
            if (~write_buffer_stall) 
            begin
                if (~write_buffer_need_reg)
                // if (~(valid & dirty) ) 
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

module Icache(
    input          clk,
    // processor interface
    input          proc_reset,
    input          proc_read,
    input          proc_write,
    input   [29:0] proc_addr,   // 28-bit address + 2-bit offset
    input   [31:0] proc_wdata,
    output         proc_stall,
    output  [31:0] proc_rdata,
    // memory interface
    input  [127:0] mem_rdata,
    input          mem_ready,
    output         mem_read,
    output         mem_write,
    output  [27:0] mem_addr,
    output [127:0] mem_wdata
);

// -----------------------------------------------------------------------------
// parameters & variables
// -----------------------------------------------------------------------------

parameter NUM_BLOCKS = 8;
parameter BLOCK_SIZE = 4;

parameter S_SERVE = 0;  // read hit, serve data without stalling
parameter S_RMISS = 1;  // read miss

integer i, j;

// -----------------------------------------------------------------------------
// signal declarations
// -----------------------------------------------------------------------------

// FSM
reg  state, next_state;

// address decoding
wire  [1:0] addr_offset;  // 2-bit
wire  [2:0] addr_block;   // 3-bit
wire [24:0] addr_tag;     // 25-bit

// cache registers
reg         cache_valid      [0:NUM_BLOCKS-1];  // valid bits
reg         next_cache_valid [0:NUM_BLOCKS-1];
reg  [24:0] cache_tag        [0:NUM_BLOCKS-1];  // 28 - 3 = 25 bits
reg  [24:0] next_cache_tag   [0:NUM_BLOCKS-1];
reg  [31:0] cache_data       [0:NUM_BLOCKS-1][0:BLOCK_SIZE-1];
reg  [31:0] next_cache_data  [0:NUM_BLOCKS-1][0:BLOCK_SIZE-1];

// cache read multiplexing
reg         cache_block_valid;
reg  [24:0] cache_block_tag;
reg  [31:0] cache_block_data [0:BLOCK_SIZE-1];
reg  [31:0] cache_read_data;

// control signals
wire hit;
reg  mem_ready_reg ;
reg [127:0]  mem_rdata_reg; 


always @(posedge clk or posedge proc_reset) begin
    if(proc_reset)begin
        mem_rdata_reg <= 128'd0;
        mem_ready_reg <= 1'd0;
    end else begin
        mem_rdata_reg <= mem_rdata;
        mem_ready_reg <= mem_ready;
    end
end
// -----------------------------------------------------------------------------
// outputs
// -----------------------------------------------------------------------------

assign proc_stall = !(
    state == S_SERVE && (next_state == S_SERVE )
);
assign proc_rdata = cache_read_data;

assign mem_read = (mem_ready) ? 1'b0 :(state == S_RMISS);
assign mem_write = 0;
assign mem_addr = proc_addr[29:2];
assign mem_wdata =0;

// -----------------------------------------------------------------------------
// FSM
// -----------------------------------------------------------------------------

// next_state
always @(*) begin
    case (state)
        S_SERVE: begin
            if (proc_read) begin
                next_state = (hit) ? S_SERVE : S_RMISS;
            end
            else begin  // no data access
                next_state = S_SERVE;
            end
        end
        S_RMISS: begin
            if (mem_ready_reg) begin
                next_state =  S_SERVE;
            end
            else next_state = S_RMISS;
        end
        default: next_state = S_SERVE;
    endcase
end

always @(posedge clk) begin
    if (proc_reset) state <= S_SERVE;
    else            state <= next_state;
end

// -----------------------------------------------------------------------------
// combinational part 
// ----------------------------------------------------------------------------- 

// control signals
assign hit = cache_block_valid && (cache_block_tag == addr_tag);

// address decoding
assign addr_offset = proc_addr[1:0];
assign addr_block  = proc_addr[4:2];
assign addr_tag    = proc_addr[29:5];

// cache read multiplexing
always @(*) begin
    cache_block_valid = cache_valid[addr_block];
    cache_block_tag = cache_tag[addr_block];
    for (j=0; j<BLOCK_SIZE; j=j+1) 
        cache_block_data[j] = cache_data[addr_block][j];
end

always @(*) cache_read_data = cache_block_data[addr_offset];

// next_cache_valid
always @(*) begin
    // default
    for (i=0; i<NUM_BLOCKS; i=i+1) begin
        next_cache_valid[i] = cache_valid[i];
    end

    if (state == S_RMISS && mem_ready_reg)
        next_cache_valid[addr_block] = 1;
end

// next_cache_tag
always @(*) begin
    // default
    for (i=0; i<NUM_BLOCKS; i=i+1) begin
        next_cache_tag[i] = cache_tag[i];
    end

    if (state == S_RMISS && mem_ready_reg)
        next_cache_tag[addr_block] = addr_tag;
end

// next_cache_data
always @(*) begin
    // default
    for (i=0; i<NUM_BLOCKS; i=i+1) begin
        for (j=0; j<BLOCK_SIZE; j=j+1) begin
            next_cache_data[i][j] = cache_data[i][j];
        end
    end

   if (state == S_RMISS && mem_ready_reg) begin
       
        // read miss
       
	    for (j=0; j<BLOCK_SIZE; j=j+1) begin
		next_cache_data[addr_block][j] = mem_rdata_reg[j*32+:32];
	    end
        
    end
end

// ----------------------------------------------------------------------------- 
// sequential part
// -----------------------------------------------------------------------------

always @(posedge clk) begin
    if (proc_reset) begin
      
        for (i=0; i<NUM_BLOCKS; i=i+1) begin
            cache_valid[i] <= 0;
            cache_tag[i] <= 0;
            for (j=0; j<BLOCK_SIZE; j=j+1) begin
                cache_data[i][j] <= 0;
            end
        end
    end
    else begin
   
        for (i=0; i<NUM_BLOCKS; i=i+1) begin
            cache_valid[i] <= next_cache_valid[i];
            cache_tag[i] <= next_cache_tag[i];
            for (j=0; j<BLOCK_SIZE; j=j+1) begin
                cache_data[i][j] <= next_cache_data[i][j];
            end
        end
    end
end

endmodule