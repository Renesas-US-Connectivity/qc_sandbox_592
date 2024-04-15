import os
import sys
import shlex

print("Converting JSON to 'C' Array...")

input_file = sys.argv[1]
output_file = sys.argv[2]

output_full_name = os.path.basename(output_file)
output_file_name = os.path.splitext(output_full_name)

print(output_full_name)
print(output_file_name)

try:
    f_in = open(input_file, "r")
    
    try:
        f_out = open(output_file, "w")
    
        try:
            print("Writing header to " + output_file)
            f_out.write("#ifndef " + output_file_name[0].upper() + "_H\n")
            f_out.write("#define " + output_file_name[0].upper() + "_H\n")
            f_out.write("#include <stdint.h>\n")
            f_out.write("const uint8_t " + output_file_name[0] + "[] = {\n")
            
            print("Writing JSON data... ")
            out_line_len = 0
            bytes = 0
            while True:
                line = f_in.readline()
                
                if not line:
                    break
                
                line_tokens = shlex.split(line, posix=False)
                separator = "";
                line = separator.join(line_tokens)
                                
                i = 0
                while i < len(line):
                    if line[i] != '\r' and line[i] != '\n':                 
                        f_out.write(hex(ord(line[i])) + ", ")
                        bytes += 1
                        out_line_len += 1
                        if out_line_len == 16:
                            f_out.write("\n")
                            out_line_len = 0            
                    i += 1
                        
            bytes += 1            
            f_out.write("0x00 };\n")
            f_out.write("#endif\n")
            print("Finished - Converted to array containing {} bytes\n" .format(bytes))
            
        except:
            print("ERROR: Unable to write to output file - " + output_file)
		
    except:
        print("ERROR: Unable to open output file - " + output_file)

    finally:
        f_in.close()
        f_out.close()

except:
    print("ERROR: Unable to open input file - " + input_file)
