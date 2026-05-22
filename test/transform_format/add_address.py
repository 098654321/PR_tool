import xlrd

# address_file_path = "D:/mypy/xinzhai_address/0223_APB_address_all16blocks_revise.xls"
address_file_path = "./0401_Added_padctrl_configuration _APB_address_all16blocks_revise.xls"

xlrdfile = xlrd.open_workbook(address_file_path)
address_dict = dict()
for i in range(4):
    table = xlrdfile.sheets()[i]
    maxrow = table.nrows
    base_address = int(table.cell_value(1,0),16)
    row = 1
    while(row<maxrow):
        #debug
        # if row==362:
        #     debug = True
        offset_address = table.cell_value(row,1)
        regname = table.cell_value(row,2)
        if(regname ==""):
            row+=1
            continue
        # print(row," ",offset_address," ",regname)
        final_address = str(hex(base_address+int(offset_address,16)))
        final_address = "32\'h"+final_address[2:]
        address_dict[regname]= final_address
        row+=1

# path = "D:/mypy/xinzhai_address/"
#path定义了作为输入的原始contrlbit路径
path = "./regnamecontrolbit_4part/"
reg_control_bit_files = [
    path+"botleft_controlbit.txt",
    path+"botright_controlbit.txt",
    path+"topleft_controlbit.txt",
    path+"topright_controlbit.txt"
]
#outputpath定义了作为输出的包含寄存器地址的整理后的controlbit路径
outputpath = "./"
reg_address_control_bit_files = [
    outputpath+"xinzhai_address/botleft_REG0.txt",
    outputpath+"xinzhai_address/botright_REG1.txt",
    outputpath+"xinzhai_address/topleft_REG2.txt",
    outputpath+"xinzhai_address/topright_REG3.txt"
]

for i in range(4):
    readfile_path = reg_control_bit_files[i]
    writefile_path = reg_address_control_bit_files[i]
    readfile = open(readfile_path,"r")
    writefile = open(writefile_path,"w")
    for line in readfile:
        line = line.strip()
        controlbit,regname = line.split(" ")
        if not(regname in address_dict.keys()):
            print("can't find reg name : "+regname)
            exit(1)
        else:
            writefile.write(controlbit+" "+address_dict[regname]+" "+regname+"\n")