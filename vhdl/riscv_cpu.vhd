library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- ============================================================================
-- CPU RISC-V RV32I - pipeline de 5 estagios (IF ID EX MEM WB)
--
-- Suporta: add addi auipc sub | and andi or ori xor xori
--          sll slli srl srli   | lw lui sw | jal jalr beq bne
--
-- Recursos:
--   * Forwarding EX/MEM e MEM/WB + write-through no banco de registradores
--   * Stall de load-use (1 bolha)
--   * Branches e saltos (jal/jalr) resolvidos no estagio EX (flush de 2 instr)
--   * load_enable_i congela TODO o estado (carga assincrona de memoria)
--   * reset_i assincrono
--   * saidas de depuracao
--
-- Interface externa: memorias de instrucao/dados e sinais de depuracao.
-- ============================================================================
entity riscv_cpu is
    Port (
        clk_i              : in  STD_LOGIC;
        reset_i            : in  STD_LOGIC;
        load_enable_i      : in  STD_LOGIC;        -- '1' congela o pipeline (carga)
        imem_data_i        : in  STD_LOGIC_VECTOR(31 downto 0);
        dmem_rdata_i       : in  STD_LOGIC_VECTOR(31 downto 0);
        reg_sel_i          : in  STD_LOGIC_VECTOR(4 downto 0);

        imem_addr_o        : out STD_LOGIC_VECTOR(31 downto 0);
        dmem_addr_o        : out STD_LOGIC_VECTOR(31 downto 0);
        dmem_wdata_o       : out STD_LOGIC_VECTOR(31 downto 0);
        dmem_we_o          : out STD_LOGIC;

        pc_debug_o         : out STD_LOGIC_VECTOR(31 downto 0);
        instr_debug_o      : out STD_LOGIC_VECTOR(31 downto 0);
        alu_result_debug_o : out STD_LOGIC_VECTOR(31 downto 0);
        reg_debug_o        : out STD_LOGIC_VECTOR(31 downto 0);
        hazard_stall_o     : out STD_LOGIC;
        hazard_flush_o     : out STD_LOGIC
    );
end riscv_cpu;

architecture Structural of riscv_cpu is

    -- ==================== componentes ====================
    component program_counter is
        Port ( clk_i : in STD_LOGIC; reset_i : in STD_LOGIC; we_i : in STD_LOGIC;
               din_i : in STD_LOGIC_VECTOR(31 downto 0);
               dout_o : out STD_LOGIC_VECTOR(31 downto 0));
    end component;

    component pipeline_reg_IF_ID is
        Port ( clk_i : in STD_LOGIC; reset_i : in STD_LOGIC; we_i : in STD_LOGIC;
               flush_i : in STD_LOGIC;
               pc_i : in STD_LOGIC_VECTOR(31 downto 0);
               instr_i : in STD_LOGIC_VECTOR(31 downto 0);
               pc_o : out STD_LOGIC_VECTOR(31 downto 0);
               instr_o : out STD_LOGIC_VECTOR(31 downto 0));
    end component;

    component instruction_decoder is
        Port ( instr_i : in STD_LOGIC_VECTOR(31 downto 0);
               rs1_o : out STD_LOGIC_VECTOR(4 downto 0);
               rs2_o : out STD_LOGIC_VECTOR(4 downto 0);
               rd_o  : out STD_LOGIC_VECTOR(4 downto 0);
               opcode_o : out STD_LOGIC_VECTOR(6 downto 0);
               funct3_o : out STD_LOGIC_VECTOR(2 downto 0);
               funct7_o : out STD_LOGIC_VECTOR(6 downto 0);
               imm_o : out STD_LOGIC_VECTOR(31 downto 0));
    end component;

    component control_unit is
        Port ( opcode_i : in STD_LOGIC_VECTOR(6 downto 0);
               funct3_i : in STD_LOGIC_VECTOR(2 downto 0);
               funct7_i : in STD_LOGIC_VECTOR(6 downto 0);
               ALUSrc_o : out STD_LOGIC; MemtoReg_o : out STD_LOGIC;
               RegWrite_o : out STD_LOGIC; MemRead_o : out STD_LOGIC;
               MemWrite_o : out STD_LOGIC; Branch_o : out STD_LOGIC;
               Jump_o : out STD_LOGIC; Jalr_o : out STD_LOGIC;
               Lui_o : out STD_LOGIC; Auipc_o : out STD_LOGIC;
               ALUCtrl_o : out STD_LOGIC_VECTOR(3 downto 0));
    end component;

    component register_file is
        Port ( clk : in STD_LOGIC; we : in STD_LOGIC;
               rs1 : in STD_LOGIC_VECTOR(4 downto 0);
               rs2 : in STD_LOGIC_VECTOR(4 downto 0);
               rd  : in STD_LOGIC_VECTOR(4 downto 0);
               din : in STD_LOGIC_VECTOR(31 downto 0);
               dout1 : out STD_LOGIC_VECTOR(31 downto 0);
               dout2 : out STD_LOGIC_VECTOR(31 downto 0));
    end component;

    component hazard_unit is
        Port ( IF_ID_Rs1_i : in STD_LOGIC_VECTOR(4 downto 0);
               IF_ID_Rs2_i : in STD_LOGIC_VECTOR(4 downto 0);
               ID_EX_Rd_i : in STD_LOGIC_VECTOR(4 downto 0);
               ID_EX_MemRead_i : in STD_LOGIC;
               PCWrite_o : out STD_LOGIC; IF_ID_Write_o : out STD_LOGIC;
               ControlMux_o : out STD_LOGIC);
    end component;

    component pipeline_reg_ID_EX is
        Port ( clk_i : in STD_LOGIC; reset_i : in STD_LOGIC; we_i : in STD_LOGIC;
               flush_i : in STD_LOGIC;
               RegWrite_i : in STD_LOGIC; MemtoReg_i : in STD_LOGIC;
               MemRead_i : in STD_LOGIC; MemWrite_i : in STD_LOGIC;
               Branch_i : in STD_LOGIC; Jump_i : in STD_LOGIC;
               Jalr_i : in STD_LOGIC; Lui_i : in STD_LOGIC; Auipc_i : in STD_LOGIC;
               ALUCtrl_i : in STD_LOGIC_VECTOR(3 downto 0); ALUSrc_i : in STD_LOGIC;
               pc_i : in STD_LOGIC_VECTOR(31 downto 0);
               reg_data1_i : in STD_LOGIC_VECTOR(31 downto 0);
               reg_data2_i : in STD_LOGIC_VECTOR(31 downto 0);
               imm_i : in STD_LOGIC_VECTOR(31 downto 0);
               funct3_i : in STD_LOGIC_VECTOR(2 downto 0);
               rs1_i : in STD_LOGIC_VECTOR(4 downto 0);
               rs2_i : in STD_LOGIC_VECTOR(4 downto 0);
               rd_i : in STD_LOGIC_VECTOR(4 downto 0);
               RegWrite_o : out STD_LOGIC; MemtoReg_o : out STD_LOGIC;
               MemRead_o : out STD_LOGIC; MemWrite_o : out STD_LOGIC;
               Branch_o : out STD_LOGIC; Jump_o : out STD_LOGIC;
               Jalr_o : out STD_LOGIC; Lui_o : out STD_LOGIC; Auipc_o : out STD_LOGIC;
               ALUCtrl_o : out STD_LOGIC_VECTOR(3 downto 0); ALUSrc_o : out STD_LOGIC;
               pc_o : out STD_LOGIC_VECTOR(31 downto 0);
               reg_data1_o : out STD_LOGIC_VECTOR(31 downto 0);
               reg_data2_o : out STD_LOGIC_VECTOR(31 downto 0);
               imm_o : out STD_LOGIC_VECTOR(31 downto 0);
               funct3_o : out STD_LOGIC_VECTOR(2 downto 0);
               rs1_o : out STD_LOGIC_VECTOR(4 downto 0);
               rs2_o : out STD_LOGIC_VECTOR(4 downto 0);
               rd_o : out STD_LOGIC_VECTOR(4 downto 0));
    end component;

    component forwarding_unit is
        Port ( ID_EX_Rs1_i : in STD_LOGIC_VECTOR(4 downto 0);
               ID_EX_Rs2_i : in STD_LOGIC_VECTOR(4 downto 0);
               EX_MEM_Rd_i : in STD_LOGIC_VECTOR(4 downto 0);
               MEM_WB_Rd_i : in STD_LOGIC_VECTOR(4 downto 0);
               EX_MEM_RegWrite_i : in STD_LOGIC; MEM_WB_RegWrite_i : in STD_LOGIC;
               ForwardA_o : out STD_LOGIC_VECTOR(1 downto 0);
               ForwardB_o : out STD_LOGIC_VECTOR(1 downto 0));
    end component;

    component branch_comparator is
        Port ( operand_a_i : in STD_LOGIC_VECTOR(31 downto 0);
               operand_b_i : in STD_LOGIC_VECTOR(31 downto 0);
               funct3_i : in STD_LOGIC_VECTOR(2 downto 0);
               branch_o : out STD_LOGIC);
    end component;

    component alu is
        Port ( alu_op_i : in STD_LOGIC_VECTOR(3 downto 0);
               operand_a_i : in STD_LOGIC_VECTOR(31 downto 0);
               operand_b_i : in STD_LOGIC_VECTOR(31 downto 0);
               result_o : out STD_LOGIC_VECTOR(31 downto 0);
               zero_o : out STD_LOGIC; carry_debug_o : out STD_LOGIC);
    end component;

    component pipeline_reg_EX_MEM is
        Port ( clk_i : in STD_LOGIC; reset_i : in STD_LOGIC; we_i : in STD_LOGIC;
               RegWrite_i : in STD_LOGIC; MemtoReg_i : in STD_LOGIC;
               MemRead_i : in STD_LOGIC; MemWrite_i : in STD_LOGIC;
               alu_result_i : in STD_LOGIC_VECTOR(31 downto 0);
               reg_data2_i : in STD_LOGIC_VECTOR(31 downto 0);
               rd_i : in STD_LOGIC_VECTOR(4 downto 0);
               RegWrite_o : out STD_LOGIC; MemtoReg_o : out STD_LOGIC;
               MemRead_o : out STD_LOGIC; MemWrite_o : out STD_LOGIC;
               alu_result_o : out STD_LOGIC_VECTOR(31 downto 0);
               reg_data2_o : out STD_LOGIC_VECTOR(31 downto 0);
               rd_o : out STD_LOGIC_VECTOR(4 downto 0));
    end component;

    component pipeline_reg_MEM_WB is
        Port ( clk_i : in STD_LOGIC; reset_i : in STD_LOGIC; we_i : in STD_LOGIC;
               RegWrite_i : in STD_LOGIC; MemtoReg_i : in STD_LOGIC;
               read_data_i : in STD_LOGIC_VECTOR(31 downto 0);
               alu_result_i : in STD_LOGIC_VECTOR(31 downto 0);
               rd_i : in STD_LOGIC_VECTOR(4 downto 0);
               RegWrite_o : out STD_LOGIC; MemtoReg_o : out STD_LOGIC;
               read_data_o : out STD_LOGIC_VECTOR(31 downto 0);
               alu_result_o : out STD_LOGIC_VECTOR(31 downto 0);
               rd_o : out STD_LOGIC_VECTOR(4 downto 0));
    end component;

    -- ==================== sinais ====================
    signal pipe_we          : STD_LOGIC;

    -- IF
    signal pc_current       : STD_LOGIC_VECTOR(31 downto 0);
    signal pc_next          : STD_LOGIC_VECTOR(31 downto 0);
    signal pc_plus4         : STD_LOGIC_VECTOR(31 downto 0);
    signal pc_target        : STD_LOGIC_VECTOR(31 downto 0);
    signal pc_we            : STD_LOGIC;
    signal if_id_we         : STD_LOGIC;
    signal if_id_pc         : STD_LOGIC_VECTOR(31 downto 0);
    signal if_id_instr      : STD_LOGIC_VECTOR(31 downto 0);

    -- ID
    signal dec_rs1, dec_rs2, dec_rd : STD_LOGIC_VECTOR(4 downto 0);
    signal dec_opcode       : STD_LOGIC_VECTOR(6 downto 0);
    signal dec_funct3       : STD_LOGIC_VECTOR(2 downto 0);
    signal dec_funct7       : STD_LOGIC_VECTOR(6 downto 0);
    signal dec_imm          : STD_LOGIC_VECTOR(31 downto 0);

    signal ctrl_ALUSrc, ctrl_MemtoReg, ctrl_RegWrite, ctrl_MemRead : STD_LOGIC;
    signal ctrl_MemWrite, ctrl_Branch, ctrl_Jump, ctrl_Jalr        : STD_LOGIC;
    signal ctrl_Lui, ctrl_Auipc                                    : STD_LOGIC;
    signal ctrl_ALUCtrl     : STD_LOGIC_VECTOR(3 downto 0);

    -- controles apos MUX de hazard (zerados em stall)
    signal mux_ALUSrc, mux_MemtoReg, mux_RegWrite, mux_MemRead  : STD_LOGIC;
    signal mux_MemWrite, mux_Branch, mux_Jump, mux_Jalr         : STD_LOGIC;
    signal mux_Lui, mux_Auipc                                   : STD_LOGIC;
    signal mux_ALUCtrl      : STD_LOGIC_VECTOR(3 downto 0);

    signal rf_data1, rf_data2 : STD_LOGIC_VECTOR(31 downto 0);

    signal haz_pcwrite, haz_if_id_write, haz_ctrl_mux : STD_LOGIC;

    -- ID/EX
    signal id_ex_RegWrite, id_ex_MemtoReg, id_ex_MemRead, id_ex_MemWrite : STD_LOGIC;
    signal id_ex_Branch, id_ex_Jump, id_ex_Jalr, id_ex_Lui, id_ex_Auipc : STD_LOGIC;
    signal id_ex_ALUSrc     : STD_LOGIC;
    signal id_ex_ALUCtrl    : STD_LOGIC_VECTOR(3 downto 0);
    signal id_ex_pc, id_ex_data1, id_ex_data2, id_ex_imm : STD_LOGIC_VECTOR(31 downto 0);
    signal id_ex_funct3     : STD_LOGIC_VECTOR(2 downto 0);
    signal id_ex_rs1, id_ex_rs2, id_ex_rd : STD_LOGIC_VECTOR(4 downto 0);

    -- EX
    signal fw_forwardA, fw_forwardB : STD_LOGIC_VECTOR(1 downto 0);
    signal fwd_a, fwd_b     : STD_LOGIC_VECTOR(31 downto 0);
    signal alu_op_a, alu_op_b : STD_LOGIC_VECTOR(31 downto 0);
    signal alu_result       : STD_LOGIC_VECTOR(31 downto 0);
    signal alu_zero, alu_carry : STD_LOGIC;
    signal branch_taken     : STD_LOGIC;
    signal redirect         : STD_LOGIC;
    signal id_ex_pc_plus4   : STD_LOGIC_VECTOR(31 downto 0);
    signal ex_value         : STD_LOGIC_VECTOR(31 downto 0);
    signal jalr_target      : STD_LOGIC_VECTOR(31 downto 0);
    signal pcimm_target     : STD_LOGIC_VECTOR(31 downto 0);

    -- EX/MEM
    signal ex_mem_RegWrite, ex_mem_MemtoReg, ex_mem_MemRead, ex_mem_MemWrite : STD_LOGIC;
    signal ex_mem_alu_res, ex_mem_data2 : STD_LOGIC_VECTOR(31 downto 0);
    signal ex_mem_rd        : STD_LOGIC_VECTOR(4 downto 0);

    -- MEM/WB
    signal mem_wb_RegWrite, mem_wb_MemtoReg : STD_LOGIC;
    signal mem_wb_rdata, mem_wb_alu_res : STD_LOGIC_VECTOR(31 downto 0);
    signal mem_wb_rd        : STD_LOGIC_VECTOR(4 downto 0);

    -- WB
    signal wb_data          : STD_LOGIC_VECTOR(31 downto 0);

begin

    pipe_we <= NOT load_enable_i;

    -- =====================================================================
    -- IF
    -- =====================================================================
    pc_we    <= haz_pcwrite AND pipe_we;
    pc_plus4 <= std_logic_vector(unsigned(pc_current) + 4);
    pc_next  <= pc_target when redirect = '1' else pc_plus4;

    PC: program_counter port map (
        clk_i => clk_i, reset_i => reset_i, we_i => pc_we,
        din_i => pc_next, dout_o => pc_current);

    imem_addr_o <= pc_current;

    if_id_we <= haz_if_id_write AND pipe_we;

    IF_ID: pipeline_reg_IF_ID port map (
        clk_i => clk_i, reset_i => reset_i, we_i => if_id_we,
        flush_i => redirect, pc_i => pc_current, instr_i => imem_data_i,
        pc_o => if_id_pc, instr_o => if_id_instr);

    -- =====================================================================
    -- ID
    -- =====================================================================
    DEC: instruction_decoder port map (
        instr_i => if_id_instr, rs1_o => dec_rs1, rs2_o => dec_rs2, rd_o => dec_rd,
        opcode_o => dec_opcode, funct3_o => dec_funct3, funct7_o => dec_funct7,
        imm_o => dec_imm);

    CTRL: control_unit port map (
        opcode_i => dec_opcode, funct3_i => dec_funct3, funct7_i => dec_funct7,
        ALUSrc_o => ctrl_ALUSrc, MemtoReg_o => ctrl_MemtoReg, RegWrite_o => ctrl_RegWrite,
        MemRead_o => ctrl_MemRead, MemWrite_o => ctrl_MemWrite, Branch_o => ctrl_Branch,
        Jump_o => ctrl_Jump, Jalr_o => ctrl_Jalr, Lui_o => ctrl_Lui, Auipc_o => ctrl_Auipc,
        ALUCtrl_o => ctrl_ALUCtrl);

    -- MUX de hazard: em stall (haz_ctrl_mux='1') insere bolha zerando controles
    mux_ALUSrc   <= ctrl_ALUSrc   when haz_ctrl_mux = '0' else '0';
    mux_MemtoReg <= ctrl_MemtoReg when haz_ctrl_mux = '0' else '0';
    mux_RegWrite <= ctrl_RegWrite when haz_ctrl_mux = '0' else '0';
    mux_MemRead  <= ctrl_MemRead  when haz_ctrl_mux = '0' else '0';
    mux_MemWrite <= ctrl_MemWrite when haz_ctrl_mux = '0' else '0';
    mux_Branch   <= ctrl_Branch   when haz_ctrl_mux = '0' else '0';
    mux_Jump     <= ctrl_Jump     when haz_ctrl_mux = '0' else '0';
    mux_Jalr     <= ctrl_Jalr     when haz_ctrl_mux = '0' else '0';
    mux_Lui      <= ctrl_Lui      when haz_ctrl_mux = '0' else '0';
    mux_Auipc    <= ctrl_Auipc    when haz_ctrl_mux = '0' else '0';
    mux_ALUCtrl  <= ctrl_ALUCtrl  when haz_ctrl_mux = '0' else "0000";

    RF: register_file port map (
        clk => clk_i, we => mem_wb_RegWrite,
        rs1 => dec_rs1, rs2 => dec_rs2, rd => mem_wb_rd, din => wb_data,
        dout1 => rf_data1, dout2 => rf_data2);

    HAZ: hazard_unit port map (
        IF_ID_Rs1_i => dec_rs1, IF_ID_Rs2_i => dec_rs2,
        ID_EX_Rd_i => id_ex_rd, ID_EX_MemRead_i => id_ex_MemRead,
        PCWrite_o => haz_pcwrite, IF_ID_Write_o => haz_if_id_write,
        ControlMux_o => haz_ctrl_mux);

    ID_EX: pipeline_reg_ID_EX port map (
        clk_i => clk_i, reset_i => reset_i, we_i => pipe_we, flush_i => redirect,
        RegWrite_i => mux_RegWrite, MemtoReg_i => mux_MemtoReg, MemRead_i => mux_MemRead,
        MemWrite_i => mux_MemWrite, Branch_i => mux_Branch, Jump_i => mux_Jump,
        Jalr_i => mux_Jalr, Lui_i => mux_Lui, Auipc_i => mux_Auipc,
        ALUCtrl_i => mux_ALUCtrl, ALUSrc_i => mux_ALUSrc,
        pc_i => if_id_pc, reg_data1_i => rf_data1, reg_data2_i => rf_data2,
        imm_i => dec_imm, funct3_i => dec_funct3,
        rs1_i => dec_rs1, rs2_i => dec_rs2, rd_i => dec_rd,
        RegWrite_o => id_ex_RegWrite, MemtoReg_o => id_ex_MemtoReg, MemRead_o => id_ex_MemRead,
        MemWrite_o => id_ex_MemWrite, Branch_o => id_ex_Branch, Jump_o => id_ex_Jump,
        Jalr_o => id_ex_Jalr, Lui_o => id_ex_Lui, Auipc_o => id_ex_Auipc,
        ALUCtrl_o => id_ex_ALUCtrl, ALUSrc_o => id_ex_ALUSrc,
        pc_o => id_ex_pc, reg_data1_o => id_ex_data1, reg_data2_o => id_ex_data2,
        imm_o => id_ex_imm, funct3_o => id_ex_funct3,
        rs1_o => id_ex_rs1, rs2_o => id_ex_rs2, rd_o => id_ex_rd);

    -- =====================================================================
    -- EX
    -- =====================================================================
    FWD: forwarding_unit port map (
        ID_EX_Rs1_i => id_ex_rs1, ID_EX_Rs2_i => id_ex_rs2,
        EX_MEM_Rd_i => ex_mem_rd, MEM_WB_Rd_i => mem_wb_rd,
        EX_MEM_RegWrite_i => ex_mem_RegWrite, MEM_WB_RegWrite_i => mem_wb_RegWrite,
        ForwardA_o => fw_forwardA, ForwardB_o => fw_forwardB);

    -- forwarding operando A (rs1)
    with fw_forwardA select
        fwd_a <= ex_mem_alu_res when "10",
                 wb_data        when "01",
                 id_ex_data1    when others;

    -- forwarding operando B (rs2)
    with fw_forwardB select
        fwd_b <= ex_mem_alu_res when "10",
                 wb_data        when "01",
                 id_ex_data2    when others;

    -- MUX operando A: lui -> 0 ; auipc -> PC ; caso contrario -> rs1 (com fwd)
    alu_op_a <= (others => '0') when id_ex_Lui   = '1' else
                id_ex_pc        when id_ex_Auipc = '1' else
                fwd_a;

    -- MUX ALUSrc: imediato ou rs2 (com fwd)
    alu_op_b <= id_ex_imm when id_ex_ALUSrc = '1' else fwd_b;

    ALU_UNIT: alu port map (
        alu_op_i => id_ex_ALUCtrl, operand_a_i => alu_op_a, operand_b_i => alu_op_b,
        result_o => alu_result, zero_o => alu_zero, carry_debug_o => alu_carry);

    -- comparador de branch usa rs1/rs2 com forwarding (sem MUX de imediato)
    BRANCH: branch_comparator port map (
        operand_a_i => fwd_a, operand_b_i => fwd_b,
        funct3_i => id_ex_funct3, branch_o => branch_taken);

    -- valor gravado no rd: para jal/jalr e' o endereco de retorno (PC+4)
    id_ex_pc_plus4 <= std_logic_vector(unsigned(id_ex_pc) + 4);
    ex_value <= id_ex_pc_plus4 when id_ex_Jump = '1' else alu_result;

    -- alvos de desvio
    pcimm_target <= std_logic_vector(unsigned(id_ex_pc) + unsigned(id_ex_imm));
    jalr_target  <= std_logic_vector((unsigned(fwd_a) + unsigned(id_ex_imm)) and
                                     to_unsigned(16#FFFFFFFE#, 32));
    pc_target    <= jalr_target when id_ex_Jalr = '1' else pcimm_target;

    -- redireciona o PC: branch tomado OU salto incondicional
    redirect <= (id_ex_Branch AND branch_taken) OR id_ex_Jump;

    EX_MEM: pipeline_reg_EX_MEM port map (
        clk_i => clk_i, reset_i => reset_i, we_i => pipe_we,
        RegWrite_i => id_ex_RegWrite, MemtoReg_i => id_ex_MemtoReg,
        MemRead_i => id_ex_MemRead, MemWrite_i => id_ex_MemWrite,
        alu_result_i => ex_value, reg_data2_i => fwd_b, rd_i => id_ex_rd,
        RegWrite_o => ex_mem_RegWrite, MemtoReg_o => ex_mem_MemtoReg,
        MemRead_o => ex_mem_MemRead, MemWrite_o => ex_mem_MemWrite,
        alu_result_o => ex_mem_alu_res, reg_data2_o => ex_mem_data2, rd_o => ex_mem_rd);

    -- =====================================================================
    -- MEM
    -- =====================================================================
    dmem_addr_o  <= ex_mem_alu_res;
    dmem_wdata_o <= ex_mem_data2;
    dmem_we_o    <= ex_mem_MemWrite;

    MEM_WB: pipeline_reg_MEM_WB port map (
        clk_i => clk_i, reset_i => reset_i, we_i => pipe_we,
        RegWrite_i => ex_mem_RegWrite, MemtoReg_i => ex_mem_MemtoReg,
        read_data_i => dmem_rdata_i, alu_result_i => ex_mem_alu_res, rd_i => ex_mem_rd,
        RegWrite_o => mem_wb_RegWrite, MemtoReg_o => mem_wb_MemtoReg,
        read_data_o => mem_wb_rdata, alu_result_o => mem_wb_alu_res, rd_o => mem_wb_rd);

    -- =====================================================================
    -- WB
    -- =====================================================================
    wb_data <= mem_wb_rdata when mem_wb_MemtoReg = '1' else mem_wb_alu_res;

    -- =====================================================================
    -- Depuracao
    -- =====================================================================
    pc_debug_o         <= pc_current;
    instr_debug_o      <= if_id_instr;
    alu_result_debug_o <= alu_result;
    hazard_stall_o     <= NOT haz_pcwrite;
    hazard_flush_o     <= redirect;

    reg_debug_o <= rf_data1 when dec_rs1   = reg_sel_i else
                   rf_data2 when dec_rs2   = reg_sel_i else
                   wb_data  when mem_wb_rd = reg_sel_i else
                   (others => '0');

end Structural;
