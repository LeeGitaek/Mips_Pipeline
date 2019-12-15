#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/**************************************/

struct Control
{
    unsigned char RegDst;
    unsigned char Jump;
    unsigned char Branch;
    unsigned char MemRead;
    unsigned char MemtoReg;
    unsigned int ALUOp;
    unsigned char MemWrite;
    unsigned char ALUSrc;
    unsigned int RegWrite;
    unsigned int PCSrc;
};

struct Reg_Read
{
    unsigned int Read_data_1;
    unsigned int Read_data_2;
};

struct ALU
{
    unsigned char zero; // 1: enable, 0: disable
    unsigned int ALU_result;
};

struct Control control;
struct Reg_Read reg_read;
struct ALU alu;
unsigned int mem[64] = { 0 };
unsigned int reg[32] = { 0 };
static char b[9];
unsigned int pc = 0;
/**************************************/
void Register_Read(unsigned int read_reg_1, unsigned int read_reg_2);
void Control_Signal(unsigned int opcode);
unsigned int ALU_Control_Signal(unsigned int signal);
void ALU_func(unsigned int ALU_control, unsigned int a, unsigned int b);
unsigned int Memory_Access(unsigned char MemWrite, unsigned char MemRead, unsigned int addr, unsigned int write_data);
void Register_Write(unsigned int RegWrite, unsigned int Write_reg, unsigned int Write_data);
unsigned int Sign_Extend(unsigned int inst_16);
unsigned int Shift_Left_2(unsigned int inst);
unsigned int Add(unsigned int a, unsigned int b);
unsigned int Mux(char signal, unsigned int a_0, unsigned int b_1);
unsigned int Inst_Fetch(unsigned int read_addr);
unsigned int Sign_Extend(unsigned int inst_16);
const char *byte_to_binary(int x);
void print_reg_mem(void);
unsigned int bitsToBinary(unsigned int doomdada);

/**************************************/

int main(void)
{

    FILE *fp;
    unsigned int inst = 0;
    unsigned int inst_31_26 = 0;
    unsigned int inst_25_21 = 0;
    unsigned int inst_20_16 = 0;
    unsigned int inst_15_11 = 0;
    unsigned int inst_15_0 = 0;
    unsigned int inst_5_0 = 0;
    unsigned int inst_ext_32 = 0;
    unsigned int inst_ext_shift = 0;
    unsigned int pc_add_4 = 0;
    unsigned int pc_add = 0;
    unsigned int pc_add_inst = 0;
    unsigned int mux_result = 0;
    unsigned int ALU_control = 0;
    unsigned int inst_25_0 = 0;
    unsigned int jump_addr = 0;
    unsigned int mem_result = 0;
    unsigned int alu_get = 0;
    int total_cycle = 0;

    // register initialization
    /**************************************/
    reg[8] = 41621;
    reg[9] = 41621;
    reg[16] = 40;
    /**************************************/

    // memory initialization
    /**************************************/
    mem[40] = 3578;


    /************************磊眉 抛胶飘侩 涝仿 何盒**********************************/

    if ( !(fp = fopen("/Users/gitaeklee/CLionProjects/untitled9/5.txt", "r")) )
    {
        printf("error: file open fail !!\n");
        exit(1);
    }

    while (1)
    {

        fscanf(fp, "%x", &inst);

        printf("inst = %x\n pc = %d\n",inst,pc);

        mem[pc] = inst;
        pc = pc + 4;
        if(feof(fp)) break;
    }

    /************************磊眉 抛胶飘侩 涝仿 何盒**********************************/


    /************************OJ system 力免侩 何盒**********************************/
//
//    while (1)
//    {
//        scanf("%x", &inst);
//        if (inst == 0)
//            break;
//        mem[pc] = inst;
//        pc = pc + 4;
//    }
    /************************OJ system 力免侩 何盒**********************************/

    /**************************************/

    // control initialization
    /**************************************/
    control.RegDst = 0;
    control.Jump = 0;
    control.Branch = 0;
    control.MemRead = 0;
    control.ALUOp = 0;
    control.MemWrite = 0;
    control.ALUSrc = 0;
    control.RegWrite = 0;
    control.PCSrc = 0;
    /**************************************/

    print_reg_mem();

    printf("\n ***** Processor START !!! ***** \n");

    pc = 0;

    while(pc < 64)
    {

        // pc +4
        pc_add_4 = Add(pc,4);



        // instruction fetch

        inst = Inst_Fetch(pc);

        printf("Instruction = %08x \n", inst);


        // instruction decode
        inst_31_26 = inst >> 26;
        inst_25_21 = (inst & 0x03e00000) >> 21;
        inst_20_16 = (inst & 0x001f0000) >> 16;
        inst_15_11 = (inst & 0x0000f800) >> 11;
        inst_15_0 = inst & 0x0000ffff;
        inst_25_0 = inst & 0x03ffffff;
        inst_5_0 = ((inst)& 0x0000003F);
        Control_Signal(inst_31_26);
      //  printf("%x, %x, %x, %x, %x, %x\n", inst_31_26, inst_25_21, inst_20_16, inst_15_11, inst_15_0, inst_25_0);


        // Shift_Left_2 for branch
        /* add => r type , beq => i type , jump =>j type , lw => i type , sw => i type */
        // r type => op , rs , rt , rd , shamt , funct
        // i type => op , rs , rt , address / immediate
        // j type => op , address / immediate
        /*
         *
         * /
         *
         * / register read

        Register_read =
		// create control signal

		// create ALU control signal

		// ALU

		// memory access

		// register write

         1. instruction [31-26] 를 control signal 함수로 보낸다. ->[완료]
         2. control signal 에서 regdst 시그널을 저장 -> [완료]
         3. instruction [20-16] 과 instruction [15-11] 그리고 control.regdst 를 mux 로 보낸다.-> [완료]
         4. mux_result 를 write register 로 보낸다. [?]
         5. instruction [25-21] 은 read_register1 로 보낸다. ->[완료]
         6. instruction [20-16] 은 read_register2 로 보낸다. ->[완료]
         7. instruction [15-0]은 sign_extend 로 보내 32비트로 만든다.->[완료]
         8. sign_extend 결과를 shift left 2 로 보낸다.->[완료]
         9. instruction [5-0] 은 ALU control 로 보낸다. ( int 타입 ) ->[완료]

        10. control signal 에서 alu op 저장하고 alu control 로 보낸다.
        11. alu control 에서 로직 연산후 alu function 으로 보낸다.
        12 alu src  = read data 2 에서 나온것과 sign extend 32 결과를 mux 로 보내고 받은 결과 (ALU src )를 Alu function 에 넣는다.
         13. alu function 에서 read data 1 나온 것과 alu src 를 연산하여 결과와 0을 같이 내보낸다.

         */
        if (inst_31_26 == 2&&control.Jump==1){

            //printf("32bit j-type = op = %d | branch address %d\n", inst_31_26,  inst_25_0);

            alu.ALU_result = Add(inst_ext_shift,pc_add_4);
            mux_result = Mux(control.Jump,pc_add_4,alu.ALU_result);
            printf("branch mux result = %d\n",mux_result);
            pc=(mux_result*4);

        } else if (inst_31_26 == 4){
            alu.ALU_result = Add(inst_ext_shift,pc_add_4);
            control.PCSrc = control.Branch&alu.zero;
            mux_result = Mux(control.PCSrc,alu.ALU_result,pc_add_4);
            //printf("branch mux result = %d",mux_result);
            pc+=mux_result;
        }

       //  pc_add_4;
        Register_Read(inst_25_21,inst_20_16);
       // Register_Write(control.RegWrite,mux_result,writedata);

        inst_ext_32 = Sign_Extend(inst_15_0);
        inst_ext_shift = Shift_Left_2(inst_ext_32);
      //  printf("op code = %d , inst 5-0 : %s\n",inst_31_26,byte_to_binary(inst_5_0));



        if(inst_31_26 !=2){
            ALU_control = ALU_Control_Signal(inst_5_0);
           // printf("ALU CONTROL signal 을 받았다 : %d\n",ALU_control);

            if(inst_31_26 == 0){
                // r type
                ALU_func(ALU_control,reg_read.Read_data_1,reg_read.Read_data_2);
                mux_result = Mux(control.RegDst,inst_15_11,inst_20_16);
                Register_Write(control.RegWrite,mux_result,alu.ALU_result);

            } else if(inst_31_26 == 35){
                // i type


                ALU_func(ALU_control,reg_read.Read_data_1,inst_ext_32);

                mux_result = Mux(control.ALUSrc,inst_ext_32,reg_read.Read_data_2);
                mem_result = Memory_Access(control.MemWrite,control.MemRead,alu.ALU_result,reg_read.Read_data_2);
                reg_read.Read_data_2 = mem_result;

           //     printf("mem _result 를 받았다 : %d\n",reg_read.Read_data_2);

                mux_result = Mux(control.RegWrite,inst_20_16,inst_15_11);
            //    printf("mux result 를 받았다 : %d\n",mux_result);
                Register_Write(control.MemtoReg,mux_result,reg_read.Read_data_2);


            } else if(inst_31_26 == 43){
                ALU_func(ALU_control,reg_read.Read_data_1,inst_ext_32);

                mux_result = Mux(control.ALUSrc,inst_ext_32,reg_read.Read_data_2);
                mem_result = Memory_Access(control.MemWrite,control.MemRead,alu.ALU_result,reg_read.Read_data_2);
                reg_read.Read_data_2 = mem_result;
           //     printf("mem _result 를 받았다 : %d\n",mem_result);
                mux_result = Mux(control.RegWrite,inst_20_16,inst_15_11);
            //    printf("mux result 를 받았다 : %d\n",mux_result);
                Register_Write(control.MemtoReg,mux_result,reg_read.Read_data_2);

            }


        }


        // Memory_Access(unsigned char MemWrite, unsigned char MemRead, unsigned int addr, unsigned int write_data)
            // return unsigned int
        /********************************/

        // implementation

        /********************************/

        total_cycle++;

        // result
        /********************************/
        printf("PC : %d \n", pc);
        printf("Total cycle : %d \n", total_cycle);
        print_reg_mem();
        /********************************/

        //system("pause");
    }

    printf("\n ***** Processor END !!! ***** \n");



    return 0;
}

unsigned int Sign_Extend(unsigned int inst_16)
{
    unsigned int inst_32 = 0;
    if ((inst_16 & 0x00008000)) // minus
    {
        inst_32 = inst_16 | 0xffff0000;
    }
    else // plus
    {
        inst_32 = inst_16;
    }

    return inst_32;
}

const char *byte_to_binary(int x)
{
   // static char b[9];
    b[0] = '\0';

    int z;
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }
    return b;
}


unsigned int Inst_Fetch(unsigned int read_addr)
{
    unsigned int c = mem[read_addr];
    unsigned int destination_addr;

    unsigned int inst_31_26 =  c >> 26;
    unsigned int inst_25_21 = (c & 0x03e00000) >> 21;
    unsigned int inst_20_16 = (c & 0x001f0000) >> 16;
    unsigned int inst_15_11 = (c & 0x0000f800) >> 11;
    unsigned int inst_15_0 = c & 0x0000ffff;
    unsigned int inst_25_0 = c & 0x03ffffff;




    //  printf("%x, %x, %x, %x, %x, %x\n", inst_31_26, inst_25_21, inst_20_16, inst_15_11, inst_15_0, inst_25_0);
   // printf("32bit box = op = %d | rs = %d | rt = %d | rd = %d | shamt = %d | funct = %x\n", inst_31_26, inst_25_21, inst_20_16, inst_15_11, inst_15_0, inst_25_0);

    if(inst_31_26 == 4){
      //  beq
       // printf("instfetch itype  = op = %d | rs = %d | rt = %d | branch address %d\n", inst_31_26, inst_25_21, inst_20_16, inst_15_0);
        //Beq : if(R[rs]==R[rt]) PC=PC+1+BranchAddr
        if(reg[inst_25_21] == reg[inst_20_16]){
            read_addr += read_addr+1+inst_15_0;
        }
        printf(">>BEQ\n");
        control.ALUOp = 1; //116


    } else if (inst_31_26 == 2){
      //  jump
        //J :  PC=JumpAddr
       // printf("instfetch j-type = op = %d | branch address %d\n", inst_31_26,  inst_25_0);
       // printf("jump to %d\n",inst_25_0);
        printf(">>JUMP\n");
        control.ALUOp = NULL;


    } else if (inst_31_26 == 0){
      // add
       // reg[inst_15_11] = reg[inst_25_21]+reg[inst_20_16];
        printf(">>ADD\n");
        control.ALUOp = 2; //134
        pc+=4;

    } else if(inst_31_26 == 35){
      // lw
       // printf("instfetch i-type = op = %d | rs = %d | rt = %d | branch address %d\n", inst_31_26, inst_25_21, inst_20_16, inst_15_0);
      // //Lw: R[rt] = M[R[rs]+SignExtImm]
        //printf("lw= %d\n",reg[inst_25_21]+Sign_Extend(inst_15_0));

        //destination_addr = reg[inst_25_21]+Sign_Extend(inst_15_0);
      //  reg[inst_20_16] = mem[destination_addr];
        printf(">>LW\n");
        control.ALUOp = 0; //143
        pc+=4;

    } else if(inst_31_26 == 43){
      // sw
       // printf("instfetch i-type = op = %d | rs = %d | rt = %d | branch address %d\n", inst_31_26, inst_25_21, inst_20_16, inst_15_0);
      //Sw : M[R[rs]+SignExtImm] = R[rt]
        //printf(" sw = %d\n",inst_25_21+Sign_Extend(inst_15_0));
     //   destination_addr = reg[inst_25_21]+Sign_Extend(inst_15_0);
     //   mem[destination_addr] = reg[inst_20_16];
        printf(">>SW\n");
        control.ALUOp = 0; // 143
        pc+=4;

    }


    //112B0002 = 100010 01010 11000 00000 00000 010000/


  //  printf("read_addr = pc = %d\n",read_addr);
    return mem[read_addr];
}

void Register_Read(unsigned int read_reg_1, unsigned int read_reg_2)
{
    //printf("[REGISTER READ] read_reg_1 = %d , read_reg_2 = %d / read_data = %d, read_data2 = %d\n",read_reg_1,read_reg_2,reg[read_reg_1],reg[read_reg_2]);

    reg_read.Read_data_1 = reg[read_reg_1];
    reg_read.Read_data_2 = reg[read_reg_2];

}

void Control_Signal(unsigned int opcode)
{
    unsigned int signal_value = rand() % 1; // 0 or 1 랜덤

    // opcode to regdst

    if(opcode == 4){
        //beq
        control.RegDst = signal_value;
        control.ALUSrc = 0;
        control.MemtoReg = signal_value;
        control.RegWrite = 0;
        control.MemRead = 0;
        control.MemWrite = 0;
        control.Branch = 1;
        control.Jump = 0;

    } else if (opcode == 2){
        // jump
        control.RegDst = signal_value;
        control.ALUSrc = signal_value;
        control.MemtoReg = signal_value;
        control.RegWrite = signal_value;
        control.MemRead = signal_value;
        control.MemWrite = signal_value;
        control.Branch = signal_value;
        control.Jump = 1;

    } else if (opcode == 0){
        // add
        control.RegDst = 1;
        control.ALUSrc = 0;
        control.MemtoReg = 0;
        control.RegWrite = 1;
        control.MemRead = 0;
        control.MemWrite = 0;
        control.Branch = 0;
        control.Jump = 0;
    } else if(opcode == 35){
        // lw
        control.RegDst = 0;
        control.ALUSrc = 1;
        control.MemtoReg = 1;
        control.RegWrite = 1;
        control.MemRead = 1;
        control.MemWrite = 0;
        control.Branch = 0;
        control.Jump = 0;

    } else if(opcode == 43){
        //sw
        control.RegDst = signal_value;
        control.ALUSrc = 1;
        control.MemtoReg = signal_value;
        control.RegWrite = 0;
        control.MemRead = 0;
        control.MemWrite = 1;
        control.Branch = 0;
        control.Jump = 0;

    }
}

unsigned int bitsToBinary(unsigned int doomdada){
    if(doomdada == 48){
        return 0;
    } else {
        return 1;
    }
}
unsigned int ALU_Control_Signal(unsigned int signal)
{

    unsigned char opbits[2];
    char bits;
    int f0,f1,f2,f3;
    int gate_0_3_or = 0;
    int gate_1_2_or = 0;
    int gate_1_1_and = 0;
    int operation_zero;
    int operation_one;
    int operation_two;

   // printf("alu op = %d \n",control.ALUOp);

    // j // null
    if(control.ALUOp == 1){
        // 01 //116  // beq
        opbits[0] = 0;
        opbits[1] = 1;
    } else if((int)control.ALUOp == 0){
        // 00 // 143 // lw sw
        opbits[0] = 0;
        opbits[1] = 0;
    } else if((int)control.ALUOp == 2){
        //10 // 134 // add
        opbits[0] = 1;
        opbits[1] = 0;
    }
 //   printf("alu control alu op code = %d %d %d\n",(int)control.ALUOp,opbits[0],opbits[1]);

    switch (signal)
    {
        case 32:
            bits = byte_to_binary(32);
            break;

    }
   //printf("alu control funct bits  = %c %c %c %c\n",b[4],b[5],b[6],b[7]);
    // 3 2 1 0
    //printf("return %d %d\n",opbits[0],opbits[1]);

    if(opbits[0]==0&&opbits[1]==1){
         // beq // 110
     //   printf("return 3 %d %d\n",opbits[0],opbits[1]);
        return (1*100)+(1*10)+0;

    } else if(opbits[0]==0&&opbits[1]==0){
        // //lw sw 010
   //     printf("return 2 %d %d\n",opbits[0],opbits[1]);
         return (1*10)+0;

    } else if(opbits[0]==1){
        // rtype r 타입
        f0 = bitsToBinary((int)b[7]);
        f1 = bitsToBinary((int)b[6]);
        f2 = bitsToBinary((int)b[5]);
        f3 = bitsToBinary((int)b[4]);

   //     printf("bits  = %d %d %d %d\n",f3,f2,f1,f0);

        // op1 , f3 / or
        gate_0_3_or = f0|f3;
        operation_zero = opbits[0]&gate_0_3_or;
        // printf("operation 0: %d\n",operation_zero);

        // op1 , f2 / or
        gate_1_2_or = opbits[0]|f2;
        operation_one = gate_1_2_or;
        // printf("operation 1: %d\n",operation_one);

        //f1,op1 // and
        gate_1_1_and = opbits[0]&f1;
        operation_two = gate_1_1_and|opbits[1];
        // printf("operation 2: %d\n",operation_two);

       // printf("operation 2,1,0: %d %d %d = %d\n",operation_two,operation_one,operation_zero,(operation_two*100)+(operation_one*10)+operation_zero);


        return (operation_two*100)+(operation_one*10)+operation_zero;

    }



}



void ALU_func(unsigned int ALU_control, unsigned int a, unsigned int b)
{
    //*
    // input [31:0] a,b;
    // output reg[31:0] alu_result;
    //  zero  = (alu_result == 0)
    //
    // */
        //input alu control , read_data_1 , mux result
        //output alu.ALU_result , alu.zero
        // 010 lw sw alu op = 0  / 110 beq  alu op = 1 / 010 (add) alu op = 2
        unsigned int carryIn = 0;
        unsigned int length =  ( ALU_control==0 ) ? 1 : (int)log10(ALU_control)+1; //int len
        unsigned int alu_control_first; //alu control signal binary []
        unsigned int alu_control_second;//alu control signal binary []
        unsigned int alu_control_third; // alu control signal binary []
        unsigned int alu_signal_val; // alu control signal 10 진수

        if(length == 3){

            alu_control_third = (ALU_control/100)*4;
            alu_control_first = (ALU_control/10)*2;
            alu_control_second = (ALU_control%10)*1;
            alu_signal_val = alu_control_first+alu_control_second+alu_control_third;

        } else if(length == 2){

            alu_control_first = (ALU_control/10)*2;
            alu_control_second = (ALU_control%10)*1;
            alu_signal_val = alu_control_first+alu_control_second;

            if(alu_signal_val==2 && control.ALUOp == 2){

                // R-type
                // add

                alu.ALU_result = a+b;
                alu.zero = 1;

            } else if(alu_signal_val==2 && control.ALUOp == 0){
                // i-type
                // lw , sw

                alu.ALU_result = a+b;
                alu.zero = 1;

            }
        }


       // printf("alu_func = %d , read_data1 = %d , read_data2 = %d\n",alu_signal_val,a,b);
      //  printf("alu_result = %d , alu_zero = %d\n",alu.ALU_result,alu.zero);


}

unsigned int Memory_Access(unsigned char MemWrite, unsigned char MemRead, unsigned int addr, unsigned int write_data)
{
    //  mem_write  => lw = 0/ sw = 1 / add = 0 / beq = 0
    //  mem_read   => lw = 1 / sw = 0 / add = 0 / beq = 0
    //  addr
    //  write_data read_data2;


    //printf("memory _ access: addr = %d , write data = %d, mem_write = %d, mem_read = %d \n",addr,write_data,MemWrite,MemRead);

    if(MemRead == 1 && MemWrite == 0){
        // lw
        //    destination_addr = reg[inst_25_21]+Sign_Extend(inst_15_0);
        //        reg[inst_20_16] = mem[destination_addr];
        //write_data+addr

        return mem[addr];

    } else if(MemRead == 0 && MemWrite == 1){
        // sw
        // // destination_addr = reg[inst_25_21]+Sign_Extend(inst_15_0);
        //        //        mem[destination_addr] = reg[inst_20_16];

        return reg[addr];

    }




}

void Register_Write(unsigned int RegWrite, unsigned int Write_reg, unsigned int Write_data)
{
   // printf("reg_write = %d write_reg = %d  , write _data = %d\n ",RegWrite,Write_reg,Write_data);
    if(RegWrite == 1){
        reg[Write_reg] = Write_data;
    }
}


unsigned int Shift_Left_2(unsigned int inst)
{
    return inst << 2;
}

unsigned int Mux(char signal, unsigned int a_0, unsigned int b_1)
{
    return signal ? a_0:b_1;
}


unsigned int Add(unsigned int a, unsigned int b){

    return a+b;
}

void print_reg_mem(void)
{
    int reg_index = 0;
    int mem_index = 0;

    printf("\n===================== REGISTER =====================\n");

    for (reg_index = 0; reg_index < 8; reg_index++)
    {
        printf("reg[%02d] = %08d        reg[%02d] = %08d        reg[%02d] = %08d        reg[%02d] = %08d \n",
               reg_index, reg[reg_index], reg_index+8, reg[reg_index+8], reg_index+16, reg[reg_index+16], reg_index+24, reg[reg_index+24] );
    }

    printf("===================== REGISTER =====================\n");

    printf("\n===================== MEMORY =====================\n");

    for (mem_index = 0; mem_index < 32; mem_index = mem_index + 4)
    {
        printf("mem[%02d] = %012d        mem[%02d] = %012d \n",
               mem_index, mem[mem_index], mem_index + 32, mem[mem_index + 32]);
    }
    printf("===================== MEMORY =====================\n");
}
