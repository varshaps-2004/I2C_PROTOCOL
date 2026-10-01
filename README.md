# I2C_PROTOCOL
I2C Protocol Master and Slave  using Verilog
//master

module I2c_master(
	input clk,
	input rst,
	input start,
	input rw,
	input [6:0] addr,
	input [7:0] data_in,
	input [7:0] mem_addr,
	output reg [7:0] data_out,
	output reg busy,
	output reg done,
	inout sda,
	output reg scl);
parameter CLK_DIV=1;
reg sda_out;//stores the value master wants to send
reg sda_enb;//it works like a switch if it is 1 master is driving sda or 0 slave is driving 

assign sda=(sda_enb)?sda_out:1'bz;

reg [4:0] state;
reg [3:0] bit_cnt;
reg [7:0] shift_reg;

reg phase;
reg [1:0] rstart_phase;
reg [1:0] stop_phase;

reg [15:0] div_cnt;
wire tick=(div_cnt==CLK_DIV-1);

localparam IDLE=0,
	START=1,
	SLAVE_ADDR=2,
	ACK1=3,
	REG_ADDR=4,
	ACK2=5,
	DATA=6,
	ACK3=7,
	RSTART=8,
	SLAVE_ADDR_R=9,
	ACK4=10,
	READ_DATA=11,
	MNACK=12,
	STOP=13,
	DONE=14;

always @(posedge clk or posedge rst) begin
	if(rst)
		div_cnt<=0;
	else if(state==IDLE)
		div_cnt<=0;
	else if(tick)
		div_cnt<=0;
	else
		div_cnt<=div_cnt+1;
end

always @(posedge clk or posedge rst) begin
	if(rst) begin
		state<=IDLE;
		scl<=1;
		sda_enb<=1;
		sda_out<=1;
		busy<=0;
		done<=0;
		bit_cnt<=0;
		shift_reg<=0;
		data_out<=0;
		phase<=0;
		rstart_phase<=0;
		stop_phase<=0;
	end

	else if(state==IDLE) begin
		scl<=1;
		sda_enb<=1;
		sda_out<=1;
		done<=0;
		phase<=0;
		if(start) begin
			busy<=1;
			sda_out<=0;
			shift_reg<={addr,1'b0};
			bit_cnt<=7;
			phase<=0;
			state<=SLAVE_ADDR;
		end
	end

	else if(tick) begin
		case(state)
					
			SLAVE_ADDR: begin
				if(phase==0) begin
					scl<=0;
					sda_out<=shift_reg[bit_cnt];
					phase<=1;
				end
				else begin
					scl<=1;
					phase<=0;
					if(bit_cnt==0) state<=ACK1;
					else bit_cnt<=bit_cnt-1;
				end
		        end
			
				
			ACK1: begin
				if(phase==0) begin
					scl<=0;
					sda_enb<=0;
					phase<=1;
				end
				else begin
					scl<=1;
					phase<=0;
					shift_reg<=mem_addr;
					bit_cnt<=7;
					state<=REG_ADDR;
				end
			end

			REG_ADDR:begin
				if(phase==0) begin
					scl<=0;
					sda_enb<=1;
					sda_out<=shift_reg[bit_cnt];
					phase<=1;
				end
				else begin
					scl<=1;
					phase<=0;
					if(bit_cnt==0) state<=ACK2;
				        else bit_cnt<=bit_cnt-1;
				end
			end
			
			ACK2:begin
				if(phase==0) begin
					scl<=0;
					sda_enb<=0;
					phase<=1;
				end
				else begin
					scl<=1;
					phase<=0;
					if(rw) begin
						rstart_phase<=0;
					        state<=RSTART;
					end
					else begin
						shift_reg<=data_in;
						bit_cnt<=7;
						state<=DATA;
					end
				end
			end
			
			DATA:begin
				if(phase==0) begin
					scl<=0;
					sda_enb<=1;
					sda_out<=shift_reg[bit_cnt];
					phase<=1;
				end
				else begin
					scl<=1;
					phase<=0;
					if(bit_cnt==0) state<=ACK3;
					else bit_cnt<=bit_cnt-1;
				end
			end

			ACK3:begin
				if(phase==0) begin
					scl<=0;
					sda_enb<=0;
					phase<=1;
				end
				else begin
					scl<=1;
					phase<=0;
					stop_phase<=0;
					state<=STOP;
				end
			end
			
			RSTART:begin
				case(rstart_phase)
					0:begin
						sda_enb<=1;
						sda_out<=1;
						scl<=0;
						rstart_phase<=1;
					end
					1:begin
						scl<=1;
						rstart_phase<=2;
					end
					2:begin
						sda_out<=0;
						rstart_phase<=3;
					end
					3:begin
						scl<=0;
				               	shift_reg<={addr,1'b1};
						bit_cnt<=7;
						phase<=0;
						state<=SLAVE_ADDR_R;
					end
				endcase
			end

			SLAVE_ADDR_R:begin
				if(phase==0) begin
					scl<=0;
					sda_out<=shift_reg[bit_cnt];
					phase<=1;
				end
				else begin
					scl<=1;
					phase<=0;
					if(bit_cnt==0) state<=ACK4;
				        else bit_cnt<=bit_cnt-1;
				end
			end

			ACK4:begin
				if(phase==0) begin
					scl<=0;
				        sda_enb<=0;
					phase<=1;
				end
				else begin
					scl<=1;
					phase<=0;
					bit_cnt<=7;
				        state<=READ_DATA;
				end
			end

			READ_DATA:begin
				if(phase==0) begin
					scl<=0;
					phase<=1;
				end
				else begin
					scl<=1;
					data_out[bit_cnt]<=sda;
					phase<=0;
				        if(bit_cnt==0) state<=MNACK;
				        else bit_cnt<=bit_cnt-1;
				end
			end
			 
			MNACK:begin
				if(phase==0) begin
					scl<=0;
				        sda_enb<=1;
				        sda_out<=1;
					phase<=1;
				end
				else begin
					scl<=1;
					phase<=0;
					stop_phase<=0;
				        state<=STOP;
				end
			end

			STOP:begin
				case(stop_phase)
					0:begin
						sda_enb<=1;
						sda_out<=0;
						scl<=0;
						stop_phase<=1;
					end
					1:begin
						scl<=1;
						stop_phase<=2;
					end
					2:begin
						sda_out<=1;
						stop_phase<=3;
					end
					3:begin
						state<=DONE;
					end
				endcase
			end

			DONE:begin
				busy<=0;
				done<=1;
				state<=IDLE;
			end
			default:state<=IDLE;
		endcase
	end
end
endmodule




//slave

module slave_i2c(
       input clk,
       input rst,
       input scl,
       inout sda
);

parameter slave_addr=7'b1010000;

reg[7:0] shift_reg;
reg[3:0] bit_cnt;
reg[4:0] state;
reg sda_out;
reg sda_enb;
reg regaddr_done;

reg[7:0] mem[0:255];
reg [7:0] addr_ptr;
reg rw_bit;

reg sda_prev,scl_prev;
reg [7:0] rx_data;

wire start_detected;
wire scl_posedge;
wire scl_negedge;

assign sda=(sda_enb)?sda_out:1'bz;

localparam IDLE=0,
	RECEIVE_ADDR=2,
	ACK1=3,
	RECEIVE_REGADDR=4,
	ACK_REG=5,
	WRITE=6,
	ACK2=7,
	SEND_DATA=8,
	WAIT_MACK=9,
	STOP=10;

always @(posedge clk) begin
    sda_prev <= sda;
    scl_prev<=scl;
end

assign start_detected = (sda_prev == 1'b1) &&
                        (sda == 1'b0) &&
                        (scl == 1'b1);

assign scl_posedge=(~scl_prev) & scl;
assign scl_negedge=scl_prev & (~scl);

always @(posedge clk or posedge rst) begin
	if(rst) begin
		state<=IDLE;
		sda_enb<=0;
		sda_out<=1;
		bit_cnt<=7;
		addr_ptr<=0;
		rw_bit<=0;
		regaddr_done<=0;
	end
	
	else if(start_detected) begin
		state<=RECEIVE_ADDR;
		bit_cnt<=7;
		sda_enb<=0;
	end

	else begin
		case(state)
			IDLE:begin
			 end

			RECEIVE_ADDR:begin
				if(scl_posedge) begin
					rx_data = shift_reg;
					rx_data[bit_cnt]=sda;
					shift_reg<=rx_data;
					if(bit_cnt==0) begin
						if(rx_data[7:1]==slave_addr) begin  
							rw_bit<=rx_data[0];
							state<=ACK1;
						end
					else state<=IDLE;
				end	
					else bit_cnt<=bit_cnt-1;
				end
			end

			ACK1:begin
				if(scl_negedge) begin
					sda_enb<=1;
					sda_out<=0;
				end
				if(scl_posedge) begin
					bit_cnt<=7;
					if(rw_bit && regaddr_done) begin
						sda_enb<=1;
						sda_out<=mem[addr_ptr][7];
						state<=SEND_DATA;
					end
					else begin
						sda_enb<=0;
						state<=RECEIVE_REGADDR;
					end
				end
			end

			RECEIVE_REGADDR:begin
				if(scl_posedge) begin
					rx_data=shift_reg;
					rx_data[bit_cnt]=sda;
					shift_reg<=rx_data;
					if(bit_cnt==0) begin 
						addr_ptr<=rx_data;
						regaddr_done<=1;
						state<=ACK_REG;
					end
				       else bit_cnt<=bit_cnt-1;
				end
			end
			
			ACK_REG:begin
				if(scl_negedge) begin 
					sda_enb<=1;
					sda_out<=0;
				end
				if(scl_posedge) begin
					sda_enb<=0;
					bit_cnt<=7;
					state<=rw_bit? SEND_DATA: WRITE;
				end
			end

			WRITE:begin
				if(scl_posedge) begin 
					rx_data = shift_reg;
					rx_data[bit_cnt]=sda;
					shift_reg<=rx_data;
					if(bit_cnt==0) begin
						mem[addr_ptr]<=rx_data;
						state<=ACK2;
					end
					else bit_cnt<=bit_cnt-1;
				end
			end

			ACK2:begin
				if(scl_negedge) begin
					sda_enb<=1;
					sda_out<=0;
				end
				if(scl_posedge) begin
					sda_enb<=0;
					state<=STOP;
				end
			end

			SEND_DATA:begin
				if(scl_posedge) begin
					if(bit_cnt==0) begin
						sda_enb<=0;
						state<=WAIT_MACK;
					end
					else begin
					       	bit_cnt<=bit_cnt-1;
						sda_out<=mem[addr_ptr][bit_cnt-1];
					end
				end
			end

			WAIT_MACK:begin
				if(scl_posedge) begin
					state<=STOP;
				end
			end

			STOP:begin
				sda_enb<=0;
				bit_cnt<=7;
				regaddr_done<=0;
				state<=IDLE;
			end
			
			default:state<=IDLE;
		endcase
	end
end
endmodule


//testbench

module tb;
reg clk;
reg rst;
reg start;
reg rw;
reg [6:0] addr;
reg [7:0] data_in;
reg [7:0] mem_addr;

wire [7:0] data_out;
wire busy;
wire done;

wire scl;
wire sda;

I2c_master master_inst(clk,rst,start,rw,addr,data_in,mem_addr,data_out,busy,done,sda,scl);

slave_i2c slave_(clk,rst,scl,sda);

always #5 clk=~clk;
initial
begin
	clk=0;
	rst=1;
	start=0;
	rw=0;
	addr=7'b1010000;
	data_in=8'h00;
	mem_addr=8'h00;
	
	#20 rst=0;
	
	//write
	addr=7'b1010000;
	rw=0;
	mem_addr=8'h00;
	data_in=8'hA5;
	#10 start=1;
	#10 start=0;
	wait(done);
	#50;

	//write
	rw=0;
	addr=7'b1010000;
	mem_addr=8'h01;
	data_in=8'h3C;
	#10 start=1;
	#10 start=0;
	wait(done);
	#40;

	//read back from mem_addr 0x01
	rw=1;
	addr=7'b1010000;
	mem_addr=8'h01;
	#10 start=1;
	#10 start=0;
	wait(done);
	#40;


	//write 0x5A to mem_addr 0x02
	rw=0;
	addr=7'b1010000;
	mem_addr=8'h02;
	data_in=8'h5A;
	#10 start =1;
	#10 start=0;
	wait(done);
	#40;

//read back from mem_addr 0x02 (repeated start)
	rw=1;
	addr=7'b1010000;
	mem_addr=8'h02;
	#10 start=1;
	#10 start=0;
	wait(done);
	#50;
end
initial
	#4000 $finish;
endmodule
