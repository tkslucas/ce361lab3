risctakers_lab3: lab3.v
	/vol/eecs362/iverilog-new/bin/iverilog -o risctakers_lab3 lab3.v lab3_tb.v

run:
	./risctakers_lab3 +MEM_IN=test_two_adds.hex
