#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#define MEM_SIZE (512u * 1024u)
#define PROG_BASE 0x1000u

typedef struct CPU {
    uint32_t pc;
    uint64_t regs[32];
    uint8_t *mem;
    size_t   mem_size;
    int      halted;
} CPU;
typedef void (*instr_fn)(CPU *cpu, uint32_t instr);

typedef enum {
    OP_AND   = 0x00,
    OP_OR    = 0x01,
    OP_XOR   = 0x02,
    OP_NOT   = 0x03,
    OP_SHFTR  = 0x04,
    OP_SHFTRI = 0x05,
    OP_SHFTL  = 0x06,
    OP_SHFTLI = 0x07,
    OP_BR     = 0x08,
    OP_BRR    = 0x09,
    OP_BRR_L  = 0x0A,
    OP_BRNZ   = 0x0B,
    OP_CALL   = 0x0C,
    OP_RETURN = 0x0D,
    OP_BRGT   = 0x0E,
    OP_PRIV   = 0x0F,
    OP_MOV_MEM_RD = 0x10,
    OP_MOV_REG    = 0x11,
    OP_MOV_LIT    = 0x12,
    OP_MOV_MEM_WR = 0x13,
    OP_ADDF  = 0x14,
    OP_SUBF  = 0x15,
    OP_MULF  = 0x16,
    OP_DIVF  = 0x17,
    OP_ADD   = 0x18,
    OP_ADDI  = 0x19,
    OP_SUB   = 0x1A,
    OP_SUBI  = 0x1B,
    OP_MUL   = 0x1C,
    OP_DIV   = 0x1D,

    NUM_OPCODES = 0x20   // opcodes go 0x00–0x1D
} opcode_t;

instr_fn instr_table[NUM_OPCODES];

static inline uint32_t get_opcode(uint32_t instr) { return (instr >> 27) & 0x1F; }
static inline uint8_t get_rd(uint32_t instr) { return (instr >> 22) & 0x1F; }
static inline uint8_t get_rs(uint32_t instr) { return (instr >> 17) & 0x1F; }
static inline uint8_t get_rt(uint32_t instr) { return (instr >> 12) & 0x1F; }
static inline uint32_t get_L(uint32_t instr) { return instr & 0xFFF; }
static inline int32_t get_L_signed(uint32_t instr) {
    int32_t L = instr & 0xFFF;
    if (L & 0x800) L |= ~0xFFF;
    return L;
}

static void io_error(void) {
	fprintf(stderr, "Invalid tinker filepath\n");
	exit(1);
}
static void sim_error(void) {
	fprintf(stderr, "Simulation error\n");
	exit(1);
}

static uint32_t mem_read_u32(CPU *cpu, uint64_t addr) {
    if (addr > (uint64_t)cpu->mem_size - 4) sim_error();   // prevents wrap
    return (uint32_t)cpu->mem[addr]
         | ((uint32_t)cpu->mem[addr + 1] << 8)
         | ((uint32_t)cpu->mem[addr + 2] << 16)
         | ((uint32_t)cpu->mem[addr + 3] << 24);
}
static uint64_t mem_read_u64(CPU *cpu, uint64_t addr) {
    if (addr > (uint64_t)cpu->mem_size - 8) sim_error();
    uint64_t x = 0;
    for (int i = 0; i < 8; i++) x |= ((uint64_t)cpu->mem[addr + i]) << (8*i);
    return x;
}
static void mem_write_u64(CPU *cpu, uint64_t addr, uint64_t val) {
    if (addr > (uint64_t)cpu->mem_size - 8) sim_error();
    for (int i = 0; i < 8; i++) cpu->mem[addr + i] = (uint8_t)((val >> (8*i)) & 0xFF);
}

static void exec_illegal(CPU *cpu, uint32_t instr) {
    (void)cpu; (void)instr;
    sim_error();
}

static void exec_and(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    cpu->regs[rd] = cpu->regs[rs] & cpu->regs[rt];
}
static void exec_or(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    cpu->regs[rd] = cpu->regs[rs] | cpu->regs[rt];
}
static void exec_xor(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    cpu->regs[rd] = cpu->regs[rs] ^ cpu->regs[rt];
}
static void exec_not(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    cpu->regs[rd] = ~cpu->regs[rs];
}

static void exec_shftr(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    cpu->regs[rd] = cpu->regs[rs] >> cpu->regs[rt];
}
static void exec_shftri(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint32_t L  = get_L(instr);
    cpu->regs[rd] = cpu->regs[rd] >> L;
}
static void exec_shftl(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    cpu->regs[rd] = cpu->regs[rs] << cpu->regs[rt];
}
static void exec_shftli(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint32_t L  = get_L(instr);
    cpu->regs[rd] = cpu->regs[rd] << L;
}

static void exec_br(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    cpu->pc = cpu->regs[rd];
}
static void exec_brr(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    cpu->pc += cpu->regs[rd];
}
static void exec_brr_l(CPU *cpu, uint32_t instr) {
    int32_t L = get_L_signed(instr);
    cpu->pc += L;
}
static void exec_brnz(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    if (cpu->regs[rs] == 0) cpu->pc += 4;
    else cpu->pc = cpu->regs[rd];
}
static void exec_call(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint64_t sp = cpu->regs[31];
    if (sp < 8) sim_error();
    mem_write_u64(cpu, sp - 8, (uint64_t)(cpu->pc + 4));
    cpu->pc = (uint32_t)cpu->regs[rd];
}
static void exec_return(CPU *cpu, uint32_t instr) {
    (void)instr;
    uint64_t sp = cpu->regs[31];
    if (sp < 8) sim_error();
    cpu->pc = (uint32_t)mem_read_u64(cpu, sp - 8);
}
static void exec_brgt(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    if (cpu->regs[rs] > cpu->regs[rt]) cpu->pc = cpu->regs[rd];
    else cpu->pc += 4;
}

static void exec_halt(CPU *cpu, uint32_t instr) {
    (void)instr;
    cpu->halted = 1;
}
static void exec_input(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint64_t port = cpu->regs[rs];
    if (port != 0) { cpu->pc += 4; return; }
    uint64_t value;
    if (scanf("%" SCNu64, &value) != 1) sim_error();
    cpu->regs[rd] = value;
    cpu->pc += 4;
}
static void exec_output(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint64_t port = cpu->regs[rd];
    if (port != 1) { cpu->pc += 4; return; }
    printf("%" PRIu64 "\n", cpu->regs[rs]);
    cpu->pc += 4;
}
static void exec_priv(CPU *cpu, uint32_t instr) {
    uint32_t L = get_L(instr);
    if (L == 0x0) { exec_halt(cpu, instr); }
    else if (L == 0x3) { exec_input(cpu, instr); }
    else if (L == 0x4) { exec_output(cpu, instr); } 
    else { exec_illegal(cpu, instr); }
}

static void exec_mov_mem_rd(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    int32_t L = get_L_signed(instr);
    cpu->regs[rd] = mem_read_u64(cpu, cpu->regs[rs] + L);
}
static void exec_mov_reg(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    cpu->regs[rd] = cpu->regs[rs];
}
static void exec_mov_lit(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint64_t L = (uint64_t)(get_L(instr) & 0xFFF);
    uint64_t mask = ((uint64_t)0xFFF) << 52;
    cpu->regs[rd] = (cpu->regs[rd] & ~mask) | (L << 52);
}
static void exec_mov_mem_wr(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    int32_t L = get_L_signed(instr);
    mem_write_u64(cpu, cpu->regs[rd] + L, cpu->regs[rs]);
}

static inline double u64_to_double(uint64_t u) {
    double d;
    memcpy(&d, &u, sizeof(d));
    return d;
}
static inline uint64_t double_to_u64(double d) {
    uint64_t u;
    memcpy(&u, &d, sizeof(u));
    return u;
}
static void exec_addf(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    double a = u64_to_double(cpu->regs[rs]);
    double b = u64_to_double(cpu->regs[rt]);
    double c = a + b;
    cpu->regs[rd] = double_to_u64(c);
}
static void exec_subf(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    double a = u64_to_double(cpu->regs[rs]);
    double b = u64_to_double(cpu->regs[rt]);
    double c = a - b;
    cpu->regs[rd] = double_to_u64(c);
}
static void exec_mulf(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    double a = u64_to_double(cpu->regs[rs]);
    double b = u64_to_double(cpu->regs[rt]);
    double c = a * b;
    cpu->regs[rd] = double_to_u64(c);
}
static void exec_divf(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    double a = u64_to_double(cpu->regs[rs]);
    double b = u64_to_double(cpu->regs[rt]);
    if (b == 0.0) { sim_error(); }
    double c = a / b;
    cpu->regs[rd] = double_to_u64(c);
}

static void exec_add(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    cpu->regs[rd] = cpu->regs[rs] + cpu->regs[rt];
}
static void exec_addi(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint32_t L = get_L(instr);
    cpu->regs[rd] = cpu->regs[rd] + L;
}
static void exec_sub(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    cpu->regs[rd] = cpu->regs[rs] - cpu->regs[rt];
}
static void exec_subi(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint32_t L = get_L(instr);
    cpu->regs[rd] = cpu->regs[rd] - L;
}
static void exec_mul(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    cpu->regs[rd] = cpu->regs[rs] * cpu->regs[rt];
}
static void exec_div(CPU *cpu, uint32_t instr) {
    uint8_t rd = get_rd(instr);
    uint8_t rs = get_rs(instr);
    uint8_t rt = get_rt(instr);
    if (cpu->regs[rt] == 0) { sim_error(); }
    cpu->regs[rd] = cpu->regs[rs] / cpu->regs[rt];
}

void init_instr_table(void) {
    for (int i = 0; i < NUM_OPCODES; i++) { instr_table[i] = exec_illegal; }
    instr_table[OP_AND] = exec_and;
    instr_table[OP_OR]  = exec_or;
    instr_table[OP_XOR] = exec_xor;
    instr_table[OP_NOT] = exec_not;
    instr_table[OP_SHFTR]  = exec_shftr;
    instr_table[OP_SHFTRI] = exec_shftri;
    instr_table[OP_SHFTL]  = exec_shftl;
    instr_table[OP_SHFTLI] = exec_shftli;
    instr_table[OP_BR]     = exec_br;
    instr_table[OP_BRR]    = exec_brr;
    instr_table[OP_BRR_L]  = exec_brr_l;
    instr_table[OP_BRNZ]   = exec_brnz;
    instr_table[OP_CALL]   = exec_call;
    instr_table[OP_RETURN] = exec_return;
    instr_table[OP_BRGT]   = exec_brgt;
    instr_table[OP_PRIV] = exec_priv;
    instr_table[OP_MOV_MEM_RD] = exec_mov_mem_rd;
    instr_table[OP_MOV_REG]    = exec_mov_reg;
    instr_table[OP_MOV_LIT]    = exec_mov_lit;
    instr_table[OP_MOV_MEM_WR] = exec_mov_mem_wr;
    instr_table[OP_ADDF] = exec_addf;
    instr_table[OP_SUBF] = exec_subf;
    instr_table[OP_MULF] = exec_mulf;
    instr_table[OP_DIVF] = exec_divf;
    instr_table[OP_ADD]  = exec_add;
    instr_table[OP_ADDI] = exec_addi;
    instr_table[OP_SUB]  = exec_sub;
    instr_table[OP_SUBI] = exec_subi;
    instr_table[OP_MUL]  = exec_mul;
    instr_table[OP_DIV]  = exec_div;
}

void step(CPU *cpu, uint32_t instr) {
    uint32_t op = get_opcode(instr);
    instr_table[op](cpu, instr);
    if (!cpu->halted && !(op >= OP_BR && op <= OP_PRIV)) cpu->pc += 4;
}
static void cpu_init(CPU *cpu) {
    memset(cpu, 0, sizeof(*cpu));
    cpu->mem_size = MEM_SIZE;
    cpu->mem = (uint8_t*)calloc(cpu->mem_size, 1);
    if (!cpu->mem) sim_error();

    cpu->pc = PROG_BASE;
    cpu->regs[31] = MEM_SIZE;
    cpu->halted = 0;
}
static void load_program(CPU *cpu, FILE *f) {
    if (fseek(f, 0, SEEK_END) != 0) io_error();
    long sz = ftell(f);
    if (sz < 0) io_error();
    rewind(f);

    uint64_t max = cpu->mem_size - PROG_BASE;
    if ((uint64_t)sz > max) sim_error();

    size_t nread = fread(cpu->mem + PROG_BASE, 1, (size_t)sz, f);
    if (nread != (size_t)sz) io_error();
}
static void run(CPU *cpu) {
    while (!cpu->halted) {
        uint32_t instr = mem_read_u32(cpu, cpu->pc);
        step(cpu, instr);
    }
}

int main(int argc, char **argv) {
    if (argc != 2) io_error();

    FILE *f = fopen(argv[1], "rb");
    if (!f) io_error();

    CPU cpu;
    init_instr_table();
    cpu_init(&cpu);
    load_program(&cpu, f);
    fclose(f);

    run(&cpu);

    free(cpu.mem);
    return 0;
}