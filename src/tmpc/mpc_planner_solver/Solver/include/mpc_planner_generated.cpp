#include <mpc_planner_generated.h>

#include <ros_tools/logging.h>

#include <stdexcept>

namespace MPCPlanner{

double getForcesOutput(const Solver_output& output, const int k, const int var_index){
		if(k == 0)
			{
				if(var_index >= 2)					LOG_WARN("getForcesOutput for k = 0 returns the initial state.");
			return output.x01[var_index];
		}
		if(k == 1)
			return output.x02[var_index];
		if(k == 2)
			return output.x03[var_index];
		if(k == 3)
			return output.x04[var_index];
		if(k == 4)
			return output.x05[var_index];
		if(k == 5)
			return output.x06[var_index];
		if(k == 6)
			return output.x07[var_index];
		if(k == 7)
			return output.x08[var_index];
		if(k == 8)
			return output.x09[var_index];
		if(k == 9)
			return output.x10[var_index];
		if(k == 10)
			return output.x11[var_index];
		if(k == 11)
			return output.x12[var_index];
		if(k == 12)
			return output.x13[var_index];
		if(k == 13)
			return output.x14[var_index];
		if(k == 14)
			return output.x15[var_index];
		if(k == 15)
			return output.x16[var_index];
		if(k == 16)
			return output.x17[var_index];
		if(k == 17)
			return output.x18[var_index];
		if(k == 18)
			return output.x19[var_index];
		if(k == 19)
			return output.x20[var_index];
		if(k == 20)
			return output.x21[var_index];
		if(k == 21)
			return output.x22[var_index];
		if(k == 22)
			return output.x23[var_index];
		if(k == 23)
			return output.x24[var_index];
		if(k == 24)
			return output.x25[var_index];
		if(k == 25)
			return output.x26[var_index];
		if(k == 26)
			return output.x27[var_index];
		if(k == 27)
			return output.x28[var_index];
		if(k == 28)
			return output.x29[var_index];
		if(k == 29)
			return output.x30[var_index];
throw std::runtime_error("Invalid k value for getForcesOutput");
}

void loadForcesWarmstart(Solver_params& params, const Solver_output& output){
		for (int i = 0; i < 2; i++){
			params.z_init_00[i] = params.x0[i];
		}
		for (int i = 0; i < 7; i++){
			params.z_init_01[i] = params.x0[7*1 + i];
			params.z_init_02[i] = params.x0[7*2 + i];
			params.z_init_03[i] = params.x0[7*3 + i];
			params.z_init_04[i] = params.x0[7*4 + i];
			params.z_init_05[i] = params.x0[7*5 + i];
			params.z_init_06[i] = params.x0[7*6 + i];
			params.z_init_07[i] = params.x0[7*7 + i];
			params.z_init_08[i] = params.x0[7*8 + i];
			params.z_init_09[i] = params.x0[7*9 + i];
			params.z_init_10[i] = params.x0[7*10 + i];
			params.z_init_11[i] = params.x0[7*11 + i];
			params.z_init_12[i] = params.x0[7*12 + i];
			params.z_init_13[i] = params.x0[7*13 + i];
			params.z_init_14[i] = params.x0[7*14 + i];
			params.z_init_15[i] = params.x0[7*15 + i];
			params.z_init_16[i] = params.x0[7*16 + i];
			params.z_init_17[i] = params.x0[7*17 + i];
			params.z_init_18[i] = params.x0[7*18 + i];
			params.z_init_19[i] = params.x0[7*19 + i];
			params.z_init_20[i] = params.x0[7*20 + i];
			params.z_init_21[i] = params.x0[7*21 + i];
			params.z_init_22[i] = params.x0[7*22 + i];
			params.z_init_23[i] = params.x0[7*23 + i];
			params.z_init_24[i] = params.x0[7*24 + i];
			params.z_init_25[i] = params.x0[7*25 + i];
			params.z_init_26[i] = params.x0[7*26 + i];
			params.z_init_27[i] = params.x0[7*27 + i];
			params.z_init_28[i] = params.x0[7*28 + i];
			params.z_init_29[i] = params.x0[7*29 + i];
		}
	}
	void setForcesReinitialize(Solver_params& params, const bool value){
	}
}
