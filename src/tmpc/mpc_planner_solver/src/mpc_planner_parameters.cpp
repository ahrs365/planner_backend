#include <mpc_planner_solver/mpc_planner_parameters.h>

#include <mpc_planner_solver/solver_interface.h>
namespace MPCPlanner{

void setSolverParameterAcceleration(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 1055 + 0] = value;
}
void setSolverParameterAngularVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 1055 + 1] = value;
}
void setSolverParameterVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 1055 + 2] = value;
}
void setSolverParameterReferenceVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 1055 + 3] = value;
}
void setSolverParameterContour(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 1055 + 4] = value;
}
void setSolverParameterLag(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 1055 + 5] = value;
}
void setSolverParameterTerminalAngle(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 1055 + 6] = value;
}
void setSolverParameterTerminalContouring(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 1055 + 7] = value;
}
void setSolverParameterSplineXA(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 8] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 17] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 26] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 35] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 44] = value;
}
void setSolverParameterSplineXB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 9] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 18] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 27] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 36] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 45] = value;
}
void setSolverParameterSplineXC(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 10] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 19] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 28] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 37] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 46] = value;
}
void setSolverParameterSplineXD(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 11] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 20] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 29] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 38] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 47] = value;
}
void setSolverParameterSplineYA(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 12] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 21] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 30] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 39] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 48] = value;
}
void setSolverParameterSplineYB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 13] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 22] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 31] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 40] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 49] = value;
}
void setSolverParameterSplineYC(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 14] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 23] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 32] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 41] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 50] = value;
}
void setSolverParameterSplineYD(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 15] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 24] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 33] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 42] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 51] = value;
}
void setSolverParameterSplineStart(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 16] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 25] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 34] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 43] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 52] = value;
}
void setSolverParameterLinConstraintA1(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 53] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 56] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 59] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 62] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 65] = value;
	else if(index == 5)
		params.all_parameters[k * 1055 + 68] = value;
	else if(index == 6)
		params.all_parameters[k * 1055 + 71] = value;
	else if(index == 7)
		params.all_parameters[k * 1055 + 74] = value;
	else if(index == 8)
		params.all_parameters[k * 1055 + 77] = value;
	else if(index == 9)
		params.all_parameters[k * 1055 + 80] = value;
	else if(index == 10)
		params.all_parameters[k * 1055 + 83] = value;
	else if(index == 11)
		params.all_parameters[k * 1055 + 86] = value;
	else if(index == 12)
		params.all_parameters[k * 1055 + 89] = value;
	else if(index == 13)
		params.all_parameters[k * 1055 + 92] = value;
	else if(index == 14)
		params.all_parameters[k * 1055 + 95] = value;
	else if(index == 15)
		params.all_parameters[k * 1055 + 98] = value;
	else if(index == 16)
		params.all_parameters[k * 1055 + 101] = value;
	else if(index == 17)
		params.all_parameters[k * 1055 + 104] = value;
	else if(index == 18)
		params.all_parameters[k * 1055 + 107] = value;
	else if(index == 19)
		params.all_parameters[k * 1055 + 110] = value;
	else if(index == 20)
		params.all_parameters[k * 1055 + 113] = value;
	else if(index == 21)
		params.all_parameters[k * 1055 + 116] = value;
	else if(index == 22)
		params.all_parameters[k * 1055 + 119] = value;
	else if(index == 23)
		params.all_parameters[k * 1055 + 122] = value;
	else if(index == 24)
		params.all_parameters[k * 1055 + 125] = value;
	else if(index == 25)
		params.all_parameters[k * 1055 + 128] = value;
	else if(index == 26)
		params.all_parameters[k * 1055 + 131] = value;
	else if(index == 27)
		params.all_parameters[k * 1055 + 134] = value;
	else if(index == 28)
		params.all_parameters[k * 1055 + 137] = value;
	else if(index == 29)
		params.all_parameters[k * 1055 + 140] = value;
	else if(index == 30)
		params.all_parameters[k * 1055 + 143] = value;
	else if(index == 31)
		params.all_parameters[k * 1055 + 146] = value;
	else if(index == 32)
		params.all_parameters[k * 1055 + 149] = value;
	else if(index == 33)
		params.all_parameters[k * 1055 + 152] = value;
	else if(index == 34)
		params.all_parameters[k * 1055 + 155] = value;
	else if(index == 35)
		params.all_parameters[k * 1055 + 158] = value;
	else if(index == 36)
		params.all_parameters[k * 1055 + 161] = value;
	else if(index == 37)
		params.all_parameters[k * 1055 + 164] = value;
	else if(index == 38)
		params.all_parameters[k * 1055 + 167] = value;
	else if(index == 39)
		params.all_parameters[k * 1055 + 170] = value;
	else if(index == 40)
		params.all_parameters[k * 1055 + 173] = value;
	else if(index == 41)
		params.all_parameters[k * 1055 + 176] = value;
	else if(index == 42)
		params.all_parameters[k * 1055 + 179] = value;
	else if(index == 43)
		params.all_parameters[k * 1055 + 182] = value;
	else if(index == 44)
		params.all_parameters[k * 1055 + 185] = value;
	else if(index == 45)
		params.all_parameters[k * 1055 + 188] = value;
	else if(index == 46)
		params.all_parameters[k * 1055 + 191] = value;
	else if(index == 47)
		params.all_parameters[k * 1055 + 194] = value;
	else if(index == 48)
		params.all_parameters[k * 1055 + 197] = value;
	else if(index == 49)
		params.all_parameters[k * 1055 + 200] = value;
	else if(index == 50)
		params.all_parameters[k * 1055 + 203] = value;
	else if(index == 51)
		params.all_parameters[k * 1055 + 206] = value;
	else if(index == 52)
		params.all_parameters[k * 1055 + 209] = value;
	else if(index == 53)
		params.all_parameters[k * 1055 + 212] = value;
	else if(index == 54)
		params.all_parameters[k * 1055 + 215] = value;
	else if(index == 55)
		params.all_parameters[k * 1055 + 218] = value;
	else if(index == 56)
		params.all_parameters[k * 1055 + 221] = value;
	else if(index == 57)
		params.all_parameters[k * 1055 + 224] = value;
	else if(index == 58)
		params.all_parameters[k * 1055 + 227] = value;
	else if(index == 59)
		params.all_parameters[k * 1055 + 230] = value;
	else if(index == 60)
		params.all_parameters[k * 1055 + 233] = value;
	else if(index == 61)
		params.all_parameters[k * 1055 + 236] = value;
	else if(index == 62)
		params.all_parameters[k * 1055 + 239] = value;
	else if(index == 63)
		params.all_parameters[k * 1055 + 242] = value;
	else if(index == 64)
		params.all_parameters[k * 1055 + 245] = value;
	else if(index == 65)
		params.all_parameters[k * 1055 + 248] = value;
	else if(index == 66)
		params.all_parameters[k * 1055 + 251] = value;
	else if(index == 67)
		params.all_parameters[k * 1055 + 254] = value;
	else if(index == 68)
		params.all_parameters[k * 1055 + 257] = value;
	else if(index == 69)
		params.all_parameters[k * 1055 + 260] = value;
	else if(index == 70)
		params.all_parameters[k * 1055 + 263] = value;
	else if(index == 71)
		params.all_parameters[k * 1055 + 266] = value;
	else if(index == 72)
		params.all_parameters[k * 1055 + 269] = value;
	else if(index == 73)
		params.all_parameters[k * 1055 + 272] = value;
	else if(index == 74)
		params.all_parameters[k * 1055 + 275] = value;
	else if(index == 75)
		params.all_parameters[k * 1055 + 278] = value;
	else if(index == 76)
		params.all_parameters[k * 1055 + 281] = value;
	else if(index == 77)
		params.all_parameters[k * 1055 + 284] = value;
	else if(index == 78)
		params.all_parameters[k * 1055 + 287] = value;
	else if(index == 79)
		params.all_parameters[k * 1055 + 290] = value;
	else if(index == 80)
		params.all_parameters[k * 1055 + 293] = value;
	else if(index == 81)
		params.all_parameters[k * 1055 + 296] = value;
	else if(index == 82)
		params.all_parameters[k * 1055 + 299] = value;
	else if(index == 83)
		params.all_parameters[k * 1055 + 302] = value;
	else if(index == 84)
		params.all_parameters[k * 1055 + 305] = value;
	else if(index == 85)
		params.all_parameters[k * 1055 + 308] = value;
	else if(index == 86)
		params.all_parameters[k * 1055 + 311] = value;
	else if(index == 87)
		params.all_parameters[k * 1055 + 314] = value;
	else if(index == 88)
		params.all_parameters[k * 1055 + 317] = value;
	else if(index == 89)
		params.all_parameters[k * 1055 + 320] = value;
	else if(index == 90)
		params.all_parameters[k * 1055 + 323] = value;
	else if(index == 91)
		params.all_parameters[k * 1055 + 326] = value;
	else if(index == 92)
		params.all_parameters[k * 1055 + 329] = value;
	else if(index == 93)
		params.all_parameters[k * 1055 + 332] = value;
	else if(index == 94)
		params.all_parameters[k * 1055 + 335] = value;
	else if(index == 95)
		params.all_parameters[k * 1055 + 338] = value;
	else if(index == 96)
		params.all_parameters[k * 1055 + 341] = value;
	else if(index == 97)
		params.all_parameters[k * 1055 + 344] = value;
	else if(index == 98)
		params.all_parameters[k * 1055 + 347] = value;
	else if(index == 99)
		params.all_parameters[k * 1055 + 350] = value;
}
void setSolverParameterLinConstraintA2(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 54] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 57] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 60] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 63] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 66] = value;
	else if(index == 5)
		params.all_parameters[k * 1055 + 69] = value;
	else if(index == 6)
		params.all_parameters[k * 1055 + 72] = value;
	else if(index == 7)
		params.all_parameters[k * 1055 + 75] = value;
	else if(index == 8)
		params.all_parameters[k * 1055 + 78] = value;
	else if(index == 9)
		params.all_parameters[k * 1055 + 81] = value;
	else if(index == 10)
		params.all_parameters[k * 1055 + 84] = value;
	else if(index == 11)
		params.all_parameters[k * 1055 + 87] = value;
	else if(index == 12)
		params.all_parameters[k * 1055 + 90] = value;
	else if(index == 13)
		params.all_parameters[k * 1055 + 93] = value;
	else if(index == 14)
		params.all_parameters[k * 1055 + 96] = value;
	else if(index == 15)
		params.all_parameters[k * 1055 + 99] = value;
	else if(index == 16)
		params.all_parameters[k * 1055 + 102] = value;
	else if(index == 17)
		params.all_parameters[k * 1055 + 105] = value;
	else if(index == 18)
		params.all_parameters[k * 1055 + 108] = value;
	else if(index == 19)
		params.all_parameters[k * 1055 + 111] = value;
	else if(index == 20)
		params.all_parameters[k * 1055 + 114] = value;
	else if(index == 21)
		params.all_parameters[k * 1055 + 117] = value;
	else if(index == 22)
		params.all_parameters[k * 1055 + 120] = value;
	else if(index == 23)
		params.all_parameters[k * 1055 + 123] = value;
	else if(index == 24)
		params.all_parameters[k * 1055 + 126] = value;
	else if(index == 25)
		params.all_parameters[k * 1055 + 129] = value;
	else if(index == 26)
		params.all_parameters[k * 1055 + 132] = value;
	else if(index == 27)
		params.all_parameters[k * 1055 + 135] = value;
	else if(index == 28)
		params.all_parameters[k * 1055 + 138] = value;
	else if(index == 29)
		params.all_parameters[k * 1055 + 141] = value;
	else if(index == 30)
		params.all_parameters[k * 1055 + 144] = value;
	else if(index == 31)
		params.all_parameters[k * 1055 + 147] = value;
	else if(index == 32)
		params.all_parameters[k * 1055 + 150] = value;
	else if(index == 33)
		params.all_parameters[k * 1055 + 153] = value;
	else if(index == 34)
		params.all_parameters[k * 1055 + 156] = value;
	else if(index == 35)
		params.all_parameters[k * 1055 + 159] = value;
	else if(index == 36)
		params.all_parameters[k * 1055 + 162] = value;
	else if(index == 37)
		params.all_parameters[k * 1055 + 165] = value;
	else if(index == 38)
		params.all_parameters[k * 1055 + 168] = value;
	else if(index == 39)
		params.all_parameters[k * 1055 + 171] = value;
	else if(index == 40)
		params.all_parameters[k * 1055 + 174] = value;
	else if(index == 41)
		params.all_parameters[k * 1055 + 177] = value;
	else if(index == 42)
		params.all_parameters[k * 1055 + 180] = value;
	else if(index == 43)
		params.all_parameters[k * 1055 + 183] = value;
	else if(index == 44)
		params.all_parameters[k * 1055 + 186] = value;
	else if(index == 45)
		params.all_parameters[k * 1055 + 189] = value;
	else if(index == 46)
		params.all_parameters[k * 1055 + 192] = value;
	else if(index == 47)
		params.all_parameters[k * 1055 + 195] = value;
	else if(index == 48)
		params.all_parameters[k * 1055 + 198] = value;
	else if(index == 49)
		params.all_parameters[k * 1055 + 201] = value;
	else if(index == 50)
		params.all_parameters[k * 1055 + 204] = value;
	else if(index == 51)
		params.all_parameters[k * 1055 + 207] = value;
	else if(index == 52)
		params.all_parameters[k * 1055 + 210] = value;
	else if(index == 53)
		params.all_parameters[k * 1055 + 213] = value;
	else if(index == 54)
		params.all_parameters[k * 1055 + 216] = value;
	else if(index == 55)
		params.all_parameters[k * 1055 + 219] = value;
	else if(index == 56)
		params.all_parameters[k * 1055 + 222] = value;
	else if(index == 57)
		params.all_parameters[k * 1055 + 225] = value;
	else if(index == 58)
		params.all_parameters[k * 1055 + 228] = value;
	else if(index == 59)
		params.all_parameters[k * 1055 + 231] = value;
	else if(index == 60)
		params.all_parameters[k * 1055 + 234] = value;
	else if(index == 61)
		params.all_parameters[k * 1055 + 237] = value;
	else if(index == 62)
		params.all_parameters[k * 1055 + 240] = value;
	else if(index == 63)
		params.all_parameters[k * 1055 + 243] = value;
	else if(index == 64)
		params.all_parameters[k * 1055 + 246] = value;
	else if(index == 65)
		params.all_parameters[k * 1055 + 249] = value;
	else if(index == 66)
		params.all_parameters[k * 1055 + 252] = value;
	else if(index == 67)
		params.all_parameters[k * 1055 + 255] = value;
	else if(index == 68)
		params.all_parameters[k * 1055 + 258] = value;
	else if(index == 69)
		params.all_parameters[k * 1055 + 261] = value;
	else if(index == 70)
		params.all_parameters[k * 1055 + 264] = value;
	else if(index == 71)
		params.all_parameters[k * 1055 + 267] = value;
	else if(index == 72)
		params.all_parameters[k * 1055 + 270] = value;
	else if(index == 73)
		params.all_parameters[k * 1055 + 273] = value;
	else if(index == 74)
		params.all_parameters[k * 1055 + 276] = value;
	else if(index == 75)
		params.all_parameters[k * 1055 + 279] = value;
	else if(index == 76)
		params.all_parameters[k * 1055 + 282] = value;
	else if(index == 77)
		params.all_parameters[k * 1055 + 285] = value;
	else if(index == 78)
		params.all_parameters[k * 1055 + 288] = value;
	else if(index == 79)
		params.all_parameters[k * 1055 + 291] = value;
	else if(index == 80)
		params.all_parameters[k * 1055 + 294] = value;
	else if(index == 81)
		params.all_parameters[k * 1055 + 297] = value;
	else if(index == 82)
		params.all_parameters[k * 1055 + 300] = value;
	else if(index == 83)
		params.all_parameters[k * 1055 + 303] = value;
	else if(index == 84)
		params.all_parameters[k * 1055 + 306] = value;
	else if(index == 85)
		params.all_parameters[k * 1055 + 309] = value;
	else if(index == 86)
		params.all_parameters[k * 1055 + 312] = value;
	else if(index == 87)
		params.all_parameters[k * 1055 + 315] = value;
	else if(index == 88)
		params.all_parameters[k * 1055 + 318] = value;
	else if(index == 89)
		params.all_parameters[k * 1055 + 321] = value;
	else if(index == 90)
		params.all_parameters[k * 1055 + 324] = value;
	else if(index == 91)
		params.all_parameters[k * 1055 + 327] = value;
	else if(index == 92)
		params.all_parameters[k * 1055 + 330] = value;
	else if(index == 93)
		params.all_parameters[k * 1055 + 333] = value;
	else if(index == 94)
		params.all_parameters[k * 1055 + 336] = value;
	else if(index == 95)
		params.all_parameters[k * 1055 + 339] = value;
	else if(index == 96)
		params.all_parameters[k * 1055 + 342] = value;
	else if(index == 97)
		params.all_parameters[k * 1055 + 345] = value;
	else if(index == 98)
		params.all_parameters[k * 1055 + 348] = value;
	else if(index == 99)
		params.all_parameters[k * 1055 + 351] = value;
}
void setSolverParameterLinConstraintB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 55] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 58] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 61] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 64] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 67] = value;
	else if(index == 5)
		params.all_parameters[k * 1055 + 70] = value;
	else if(index == 6)
		params.all_parameters[k * 1055 + 73] = value;
	else if(index == 7)
		params.all_parameters[k * 1055 + 76] = value;
	else if(index == 8)
		params.all_parameters[k * 1055 + 79] = value;
	else if(index == 9)
		params.all_parameters[k * 1055 + 82] = value;
	else if(index == 10)
		params.all_parameters[k * 1055 + 85] = value;
	else if(index == 11)
		params.all_parameters[k * 1055 + 88] = value;
	else if(index == 12)
		params.all_parameters[k * 1055 + 91] = value;
	else if(index == 13)
		params.all_parameters[k * 1055 + 94] = value;
	else if(index == 14)
		params.all_parameters[k * 1055 + 97] = value;
	else if(index == 15)
		params.all_parameters[k * 1055 + 100] = value;
	else if(index == 16)
		params.all_parameters[k * 1055 + 103] = value;
	else if(index == 17)
		params.all_parameters[k * 1055 + 106] = value;
	else if(index == 18)
		params.all_parameters[k * 1055 + 109] = value;
	else if(index == 19)
		params.all_parameters[k * 1055 + 112] = value;
	else if(index == 20)
		params.all_parameters[k * 1055 + 115] = value;
	else if(index == 21)
		params.all_parameters[k * 1055 + 118] = value;
	else if(index == 22)
		params.all_parameters[k * 1055 + 121] = value;
	else if(index == 23)
		params.all_parameters[k * 1055 + 124] = value;
	else if(index == 24)
		params.all_parameters[k * 1055 + 127] = value;
	else if(index == 25)
		params.all_parameters[k * 1055 + 130] = value;
	else if(index == 26)
		params.all_parameters[k * 1055 + 133] = value;
	else if(index == 27)
		params.all_parameters[k * 1055 + 136] = value;
	else if(index == 28)
		params.all_parameters[k * 1055 + 139] = value;
	else if(index == 29)
		params.all_parameters[k * 1055 + 142] = value;
	else if(index == 30)
		params.all_parameters[k * 1055 + 145] = value;
	else if(index == 31)
		params.all_parameters[k * 1055 + 148] = value;
	else if(index == 32)
		params.all_parameters[k * 1055 + 151] = value;
	else if(index == 33)
		params.all_parameters[k * 1055 + 154] = value;
	else if(index == 34)
		params.all_parameters[k * 1055 + 157] = value;
	else if(index == 35)
		params.all_parameters[k * 1055 + 160] = value;
	else if(index == 36)
		params.all_parameters[k * 1055 + 163] = value;
	else if(index == 37)
		params.all_parameters[k * 1055 + 166] = value;
	else if(index == 38)
		params.all_parameters[k * 1055 + 169] = value;
	else if(index == 39)
		params.all_parameters[k * 1055 + 172] = value;
	else if(index == 40)
		params.all_parameters[k * 1055 + 175] = value;
	else if(index == 41)
		params.all_parameters[k * 1055 + 178] = value;
	else if(index == 42)
		params.all_parameters[k * 1055 + 181] = value;
	else if(index == 43)
		params.all_parameters[k * 1055 + 184] = value;
	else if(index == 44)
		params.all_parameters[k * 1055 + 187] = value;
	else if(index == 45)
		params.all_parameters[k * 1055 + 190] = value;
	else if(index == 46)
		params.all_parameters[k * 1055 + 193] = value;
	else if(index == 47)
		params.all_parameters[k * 1055 + 196] = value;
	else if(index == 48)
		params.all_parameters[k * 1055 + 199] = value;
	else if(index == 49)
		params.all_parameters[k * 1055 + 202] = value;
	else if(index == 50)
		params.all_parameters[k * 1055 + 205] = value;
	else if(index == 51)
		params.all_parameters[k * 1055 + 208] = value;
	else if(index == 52)
		params.all_parameters[k * 1055 + 211] = value;
	else if(index == 53)
		params.all_parameters[k * 1055 + 214] = value;
	else if(index == 54)
		params.all_parameters[k * 1055 + 217] = value;
	else if(index == 55)
		params.all_parameters[k * 1055 + 220] = value;
	else if(index == 56)
		params.all_parameters[k * 1055 + 223] = value;
	else if(index == 57)
		params.all_parameters[k * 1055 + 226] = value;
	else if(index == 58)
		params.all_parameters[k * 1055 + 229] = value;
	else if(index == 59)
		params.all_parameters[k * 1055 + 232] = value;
	else if(index == 60)
		params.all_parameters[k * 1055 + 235] = value;
	else if(index == 61)
		params.all_parameters[k * 1055 + 238] = value;
	else if(index == 62)
		params.all_parameters[k * 1055 + 241] = value;
	else if(index == 63)
		params.all_parameters[k * 1055 + 244] = value;
	else if(index == 64)
		params.all_parameters[k * 1055 + 247] = value;
	else if(index == 65)
		params.all_parameters[k * 1055 + 250] = value;
	else if(index == 66)
		params.all_parameters[k * 1055 + 253] = value;
	else if(index == 67)
		params.all_parameters[k * 1055 + 256] = value;
	else if(index == 68)
		params.all_parameters[k * 1055 + 259] = value;
	else if(index == 69)
		params.all_parameters[k * 1055 + 262] = value;
	else if(index == 70)
		params.all_parameters[k * 1055 + 265] = value;
	else if(index == 71)
		params.all_parameters[k * 1055 + 268] = value;
	else if(index == 72)
		params.all_parameters[k * 1055 + 271] = value;
	else if(index == 73)
		params.all_parameters[k * 1055 + 274] = value;
	else if(index == 74)
		params.all_parameters[k * 1055 + 277] = value;
	else if(index == 75)
		params.all_parameters[k * 1055 + 280] = value;
	else if(index == 76)
		params.all_parameters[k * 1055 + 283] = value;
	else if(index == 77)
		params.all_parameters[k * 1055 + 286] = value;
	else if(index == 78)
		params.all_parameters[k * 1055 + 289] = value;
	else if(index == 79)
		params.all_parameters[k * 1055 + 292] = value;
	else if(index == 80)
		params.all_parameters[k * 1055 + 295] = value;
	else if(index == 81)
		params.all_parameters[k * 1055 + 298] = value;
	else if(index == 82)
		params.all_parameters[k * 1055 + 301] = value;
	else if(index == 83)
		params.all_parameters[k * 1055 + 304] = value;
	else if(index == 84)
		params.all_parameters[k * 1055 + 307] = value;
	else if(index == 85)
		params.all_parameters[k * 1055 + 310] = value;
	else if(index == 86)
		params.all_parameters[k * 1055 + 313] = value;
	else if(index == 87)
		params.all_parameters[k * 1055 + 316] = value;
	else if(index == 88)
		params.all_parameters[k * 1055 + 319] = value;
	else if(index == 89)
		params.all_parameters[k * 1055 + 322] = value;
	else if(index == 90)
		params.all_parameters[k * 1055 + 325] = value;
	else if(index == 91)
		params.all_parameters[k * 1055 + 328] = value;
	else if(index == 92)
		params.all_parameters[k * 1055 + 331] = value;
	else if(index == 93)
		params.all_parameters[k * 1055 + 334] = value;
	else if(index == 94)
		params.all_parameters[k * 1055 + 337] = value;
	else if(index == 95)
		params.all_parameters[k * 1055 + 340] = value;
	else if(index == 96)
		params.all_parameters[k * 1055 + 343] = value;
	else if(index == 97)
		params.all_parameters[k * 1055 + 346] = value;
	else if(index == 98)
		params.all_parameters[k * 1055 + 349] = value;
	else if(index == 99)
		params.all_parameters[k * 1055 + 352] = value;
}
void setSolverParameterEgoDiscRadius(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 1055 + 353] = value;
}
void setSolverParameterEgoDiscOffset(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 1055 + 354] = value;
}
void setSolverParameterEllipsoidObstX(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 355] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 362] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 369] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 376] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 383] = value;
	else if(index == 5)
		params.all_parameters[k * 1055 + 390] = value;
	else if(index == 6)
		params.all_parameters[k * 1055 + 397] = value;
	else if(index == 7)
		params.all_parameters[k * 1055 + 404] = value;
	else if(index == 8)
		params.all_parameters[k * 1055 + 411] = value;
	else if(index == 9)
		params.all_parameters[k * 1055 + 418] = value;
	else if(index == 10)
		params.all_parameters[k * 1055 + 425] = value;
	else if(index == 11)
		params.all_parameters[k * 1055 + 432] = value;
	else if(index == 12)
		params.all_parameters[k * 1055 + 439] = value;
	else if(index == 13)
		params.all_parameters[k * 1055 + 446] = value;
	else if(index == 14)
		params.all_parameters[k * 1055 + 453] = value;
	else if(index == 15)
		params.all_parameters[k * 1055 + 460] = value;
	else if(index == 16)
		params.all_parameters[k * 1055 + 467] = value;
	else if(index == 17)
		params.all_parameters[k * 1055 + 474] = value;
	else if(index == 18)
		params.all_parameters[k * 1055 + 481] = value;
	else if(index == 19)
		params.all_parameters[k * 1055 + 488] = value;
	else if(index == 20)
		params.all_parameters[k * 1055 + 495] = value;
	else if(index == 21)
		params.all_parameters[k * 1055 + 502] = value;
	else if(index == 22)
		params.all_parameters[k * 1055 + 509] = value;
	else if(index == 23)
		params.all_parameters[k * 1055 + 516] = value;
	else if(index == 24)
		params.all_parameters[k * 1055 + 523] = value;
	else if(index == 25)
		params.all_parameters[k * 1055 + 530] = value;
	else if(index == 26)
		params.all_parameters[k * 1055 + 537] = value;
	else if(index == 27)
		params.all_parameters[k * 1055 + 544] = value;
	else if(index == 28)
		params.all_parameters[k * 1055 + 551] = value;
	else if(index == 29)
		params.all_parameters[k * 1055 + 558] = value;
	else if(index == 30)
		params.all_parameters[k * 1055 + 565] = value;
	else if(index == 31)
		params.all_parameters[k * 1055 + 572] = value;
	else if(index == 32)
		params.all_parameters[k * 1055 + 579] = value;
	else if(index == 33)
		params.all_parameters[k * 1055 + 586] = value;
	else if(index == 34)
		params.all_parameters[k * 1055 + 593] = value;
	else if(index == 35)
		params.all_parameters[k * 1055 + 600] = value;
	else if(index == 36)
		params.all_parameters[k * 1055 + 607] = value;
	else if(index == 37)
		params.all_parameters[k * 1055 + 614] = value;
	else if(index == 38)
		params.all_parameters[k * 1055 + 621] = value;
	else if(index == 39)
		params.all_parameters[k * 1055 + 628] = value;
	else if(index == 40)
		params.all_parameters[k * 1055 + 635] = value;
	else if(index == 41)
		params.all_parameters[k * 1055 + 642] = value;
	else if(index == 42)
		params.all_parameters[k * 1055 + 649] = value;
	else if(index == 43)
		params.all_parameters[k * 1055 + 656] = value;
	else if(index == 44)
		params.all_parameters[k * 1055 + 663] = value;
	else if(index == 45)
		params.all_parameters[k * 1055 + 670] = value;
	else if(index == 46)
		params.all_parameters[k * 1055 + 677] = value;
	else if(index == 47)
		params.all_parameters[k * 1055 + 684] = value;
	else if(index == 48)
		params.all_parameters[k * 1055 + 691] = value;
	else if(index == 49)
		params.all_parameters[k * 1055 + 698] = value;
	else if(index == 50)
		params.all_parameters[k * 1055 + 705] = value;
	else if(index == 51)
		params.all_parameters[k * 1055 + 712] = value;
	else if(index == 52)
		params.all_parameters[k * 1055 + 719] = value;
	else if(index == 53)
		params.all_parameters[k * 1055 + 726] = value;
	else if(index == 54)
		params.all_parameters[k * 1055 + 733] = value;
	else if(index == 55)
		params.all_parameters[k * 1055 + 740] = value;
	else if(index == 56)
		params.all_parameters[k * 1055 + 747] = value;
	else if(index == 57)
		params.all_parameters[k * 1055 + 754] = value;
	else if(index == 58)
		params.all_parameters[k * 1055 + 761] = value;
	else if(index == 59)
		params.all_parameters[k * 1055 + 768] = value;
	else if(index == 60)
		params.all_parameters[k * 1055 + 775] = value;
	else if(index == 61)
		params.all_parameters[k * 1055 + 782] = value;
	else if(index == 62)
		params.all_parameters[k * 1055 + 789] = value;
	else if(index == 63)
		params.all_parameters[k * 1055 + 796] = value;
	else if(index == 64)
		params.all_parameters[k * 1055 + 803] = value;
	else if(index == 65)
		params.all_parameters[k * 1055 + 810] = value;
	else if(index == 66)
		params.all_parameters[k * 1055 + 817] = value;
	else if(index == 67)
		params.all_parameters[k * 1055 + 824] = value;
	else if(index == 68)
		params.all_parameters[k * 1055 + 831] = value;
	else if(index == 69)
		params.all_parameters[k * 1055 + 838] = value;
	else if(index == 70)
		params.all_parameters[k * 1055 + 845] = value;
	else if(index == 71)
		params.all_parameters[k * 1055 + 852] = value;
	else if(index == 72)
		params.all_parameters[k * 1055 + 859] = value;
	else if(index == 73)
		params.all_parameters[k * 1055 + 866] = value;
	else if(index == 74)
		params.all_parameters[k * 1055 + 873] = value;
	else if(index == 75)
		params.all_parameters[k * 1055 + 880] = value;
	else if(index == 76)
		params.all_parameters[k * 1055 + 887] = value;
	else if(index == 77)
		params.all_parameters[k * 1055 + 894] = value;
	else if(index == 78)
		params.all_parameters[k * 1055 + 901] = value;
	else if(index == 79)
		params.all_parameters[k * 1055 + 908] = value;
	else if(index == 80)
		params.all_parameters[k * 1055 + 915] = value;
	else if(index == 81)
		params.all_parameters[k * 1055 + 922] = value;
	else if(index == 82)
		params.all_parameters[k * 1055 + 929] = value;
	else if(index == 83)
		params.all_parameters[k * 1055 + 936] = value;
	else if(index == 84)
		params.all_parameters[k * 1055 + 943] = value;
	else if(index == 85)
		params.all_parameters[k * 1055 + 950] = value;
	else if(index == 86)
		params.all_parameters[k * 1055 + 957] = value;
	else if(index == 87)
		params.all_parameters[k * 1055 + 964] = value;
	else if(index == 88)
		params.all_parameters[k * 1055 + 971] = value;
	else if(index == 89)
		params.all_parameters[k * 1055 + 978] = value;
	else if(index == 90)
		params.all_parameters[k * 1055 + 985] = value;
	else if(index == 91)
		params.all_parameters[k * 1055 + 992] = value;
	else if(index == 92)
		params.all_parameters[k * 1055 + 999] = value;
	else if(index == 93)
		params.all_parameters[k * 1055 + 1006] = value;
	else if(index == 94)
		params.all_parameters[k * 1055 + 1013] = value;
	else if(index == 95)
		params.all_parameters[k * 1055 + 1020] = value;
	else if(index == 96)
		params.all_parameters[k * 1055 + 1027] = value;
	else if(index == 97)
		params.all_parameters[k * 1055 + 1034] = value;
	else if(index == 98)
		params.all_parameters[k * 1055 + 1041] = value;
	else if(index == 99)
		params.all_parameters[k * 1055 + 1048] = value;
}
void setSolverParameterEllipsoidObstY(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 356] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 363] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 370] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 377] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 384] = value;
	else if(index == 5)
		params.all_parameters[k * 1055 + 391] = value;
	else if(index == 6)
		params.all_parameters[k * 1055 + 398] = value;
	else if(index == 7)
		params.all_parameters[k * 1055 + 405] = value;
	else if(index == 8)
		params.all_parameters[k * 1055 + 412] = value;
	else if(index == 9)
		params.all_parameters[k * 1055 + 419] = value;
	else if(index == 10)
		params.all_parameters[k * 1055 + 426] = value;
	else if(index == 11)
		params.all_parameters[k * 1055 + 433] = value;
	else if(index == 12)
		params.all_parameters[k * 1055 + 440] = value;
	else if(index == 13)
		params.all_parameters[k * 1055 + 447] = value;
	else if(index == 14)
		params.all_parameters[k * 1055 + 454] = value;
	else if(index == 15)
		params.all_parameters[k * 1055 + 461] = value;
	else if(index == 16)
		params.all_parameters[k * 1055 + 468] = value;
	else if(index == 17)
		params.all_parameters[k * 1055 + 475] = value;
	else if(index == 18)
		params.all_parameters[k * 1055 + 482] = value;
	else if(index == 19)
		params.all_parameters[k * 1055 + 489] = value;
	else if(index == 20)
		params.all_parameters[k * 1055 + 496] = value;
	else if(index == 21)
		params.all_parameters[k * 1055 + 503] = value;
	else if(index == 22)
		params.all_parameters[k * 1055 + 510] = value;
	else if(index == 23)
		params.all_parameters[k * 1055 + 517] = value;
	else if(index == 24)
		params.all_parameters[k * 1055 + 524] = value;
	else if(index == 25)
		params.all_parameters[k * 1055 + 531] = value;
	else if(index == 26)
		params.all_parameters[k * 1055 + 538] = value;
	else if(index == 27)
		params.all_parameters[k * 1055 + 545] = value;
	else if(index == 28)
		params.all_parameters[k * 1055 + 552] = value;
	else if(index == 29)
		params.all_parameters[k * 1055 + 559] = value;
	else if(index == 30)
		params.all_parameters[k * 1055 + 566] = value;
	else if(index == 31)
		params.all_parameters[k * 1055 + 573] = value;
	else if(index == 32)
		params.all_parameters[k * 1055 + 580] = value;
	else if(index == 33)
		params.all_parameters[k * 1055 + 587] = value;
	else if(index == 34)
		params.all_parameters[k * 1055 + 594] = value;
	else if(index == 35)
		params.all_parameters[k * 1055 + 601] = value;
	else if(index == 36)
		params.all_parameters[k * 1055 + 608] = value;
	else if(index == 37)
		params.all_parameters[k * 1055 + 615] = value;
	else if(index == 38)
		params.all_parameters[k * 1055 + 622] = value;
	else if(index == 39)
		params.all_parameters[k * 1055 + 629] = value;
	else if(index == 40)
		params.all_parameters[k * 1055 + 636] = value;
	else if(index == 41)
		params.all_parameters[k * 1055 + 643] = value;
	else if(index == 42)
		params.all_parameters[k * 1055 + 650] = value;
	else if(index == 43)
		params.all_parameters[k * 1055 + 657] = value;
	else if(index == 44)
		params.all_parameters[k * 1055 + 664] = value;
	else if(index == 45)
		params.all_parameters[k * 1055 + 671] = value;
	else if(index == 46)
		params.all_parameters[k * 1055 + 678] = value;
	else if(index == 47)
		params.all_parameters[k * 1055 + 685] = value;
	else if(index == 48)
		params.all_parameters[k * 1055 + 692] = value;
	else if(index == 49)
		params.all_parameters[k * 1055 + 699] = value;
	else if(index == 50)
		params.all_parameters[k * 1055 + 706] = value;
	else if(index == 51)
		params.all_parameters[k * 1055 + 713] = value;
	else if(index == 52)
		params.all_parameters[k * 1055 + 720] = value;
	else if(index == 53)
		params.all_parameters[k * 1055 + 727] = value;
	else if(index == 54)
		params.all_parameters[k * 1055 + 734] = value;
	else if(index == 55)
		params.all_parameters[k * 1055 + 741] = value;
	else if(index == 56)
		params.all_parameters[k * 1055 + 748] = value;
	else if(index == 57)
		params.all_parameters[k * 1055 + 755] = value;
	else if(index == 58)
		params.all_parameters[k * 1055 + 762] = value;
	else if(index == 59)
		params.all_parameters[k * 1055 + 769] = value;
	else if(index == 60)
		params.all_parameters[k * 1055 + 776] = value;
	else if(index == 61)
		params.all_parameters[k * 1055 + 783] = value;
	else if(index == 62)
		params.all_parameters[k * 1055 + 790] = value;
	else if(index == 63)
		params.all_parameters[k * 1055 + 797] = value;
	else if(index == 64)
		params.all_parameters[k * 1055 + 804] = value;
	else if(index == 65)
		params.all_parameters[k * 1055 + 811] = value;
	else if(index == 66)
		params.all_parameters[k * 1055 + 818] = value;
	else if(index == 67)
		params.all_parameters[k * 1055 + 825] = value;
	else if(index == 68)
		params.all_parameters[k * 1055 + 832] = value;
	else if(index == 69)
		params.all_parameters[k * 1055 + 839] = value;
	else if(index == 70)
		params.all_parameters[k * 1055 + 846] = value;
	else if(index == 71)
		params.all_parameters[k * 1055 + 853] = value;
	else if(index == 72)
		params.all_parameters[k * 1055 + 860] = value;
	else if(index == 73)
		params.all_parameters[k * 1055 + 867] = value;
	else if(index == 74)
		params.all_parameters[k * 1055 + 874] = value;
	else if(index == 75)
		params.all_parameters[k * 1055 + 881] = value;
	else if(index == 76)
		params.all_parameters[k * 1055 + 888] = value;
	else if(index == 77)
		params.all_parameters[k * 1055 + 895] = value;
	else if(index == 78)
		params.all_parameters[k * 1055 + 902] = value;
	else if(index == 79)
		params.all_parameters[k * 1055 + 909] = value;
	else if(index == 80)
		params.all_parameters[k * 1055 + 916] = value;
	else if(index == 81)
		params.all_parameters[k * 1055 + 923] = value;
	else if(index == 82)
		params.all_parameters[k * 1055 + 930] = value;
	else if(index == 83)
		params.all_parameters[k * 1055 + 937] = value;
	else if(index == 84)
		params.all_parameters[k * 1055 + 944] = value;
	else if(index == 85)
		params.all_parameters[k * 1055 + 951] = value;
	else if(index == 86)
		params.all_parameters[k * 1055 + 958] = value;
	else if(index == 87)
		params.all_parameters[k * 1055 + 965] = value;
	else if(index == 88)
		params.all_parameters[k * 1055 + 972] = value;
	else if(index == 89)
		params.all_parameters[k * 1055 + 979] = value;
	else if(index == 90)
		params.all_parameters[k * 1055 + 986] = value;
	else if(index == 91)
		params.all_parameters[k * 1055 + 993] = value;
	else if(index == 92)
		params.all_parameters[k * 1055 + 1000] = value;
	else if(index == 93)
		params.all_parameters[k * 1055 + 1007] = value;
	else if(index == 94)
		params.all_parameters[k * 1055 + 1014] = value;
	else if(index == 95)
		params.all_parameters[k * 1055 + 1021] = value;
	else if(index == 96)
		params.all_parameters[k * 1055 + 1028] = value;
	else if(index == 97)
		params.all_parameters[k * 1055 + 1035] = value;
	else if(index == 98)
		params.all_parameters[k * 1055 + 1042] = value;
	else if(index == 99)
		params.all_parameters[k * 1055 + 1049] = value;
}
void setSolverParameterEllipsoidObstPsi(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 357] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 364] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 371] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 378] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 385] = value;
	else if(index == 5)
		params.all_parameters[k * 1055 + 392] = value;
	else if(index == 6)
		params.all_parameters[k * 1055 + 399] = value;
	else if(index == 7)
		params.all_parameters[k * 1055 + 406] = value;
	else if(index == 8)
		params.all_parameters[k * 1055 + 413] = value;
	else if(index == 9)
		params.all_parameters[k * 1055 + 420] = value;
	else if(index == 10)
		params.all_parameters[k * 1055 + 427] = value;
	else if(index == 11)
		params.all_parameters[k * 1055 + 434] = value;
	else if(index == 12)
		params.all_parameters[k * 1055 + 441] = value;
	else if(index == 13)
		params.all_parameters[k * 1055 + 448] = value;
	else if(index == 14)
		params.all_parameters[k * 1055 + 455] = value;
	else if(index == 15)
		params.all_parameters[k * 1055 + 462] = value;
	else if(index == 16)
		params.all_parameters[k * 1055 + 469] = value;
	else if(index == 17)
		params.all_parameters[k * 1055 + 476] = value;
	else if(index == 18)
		params.all_parameters[k * 1055 + 483] = value;
	else if(index == 19)
		params.all_parameters[k * 1055 + 490] = value;
	else if(index == 20)
		params.all_parameters[k * 1055 + 497] = value;
	else if(index == 21)
		params.all_parameters[k * 1055 + 504] = value;
	else if(index == 22)
		params.all_parameters[k * 1055 + 511] = value;
	else if(index == 23)
		params.all_parameters[k * 1055 + 518] = value;
	else if(index == 24)
		params.all_parameters[k * 1055 + 525] = value;
	else if(index == 25)
		params.all_parameters[k * 1055 + 532] = value;
	else if(index == 26)
		params.all_parameters[k * 1055 + 539] = value;
	else if(index == 27)
		params.all_parameters[k * 1055 + 546] = value;
	else if(index == 28)
		params.all_parameters[k * 1055 + 553] = value;
	else if(index == 29)
		params.all_parameters[k * 1055 + 560] = value;
	else if(index == 30)
		params.all_parameters[k * 1055 + 567] = value;
	else if(index == 31)
		params.all_parameters[k * 1055 + 574] = value;
	else if(index == 32)
		params.all_parameters[k * 1055 + 581] = value;
	else if(index == 33)
		params.all_parameters[k * 1055 + 588] = value;
	else if(index == 34)
		params.all_parameters[k * 1055 + 595] = value;
	else if(index == 35)
		params.all_parameters[k * 1055 + 602] = value;
	else if(index == 36)
		params.all_parameters[k * 1055 + 609] = value;
	else if(index == 37)
		params.all_parameters[k * 1055 + 616] = value;
	else if(index == 38)
		params.all_parameters[k * 1055 + 623] = value;
	else if(index == 39)
		params.all_parameters[k * 1055 + 630] = value;
	else if(index == 40)
		params.all_parameters[k * 1055 + 637] = value;
	else if(index == 41)
		params.all_parameters[k * 1055 + 644] = value;
	else if(index == 42)
		params.all_parameters[k * 1055 + 651] = value;
	else if(index == 43)
		params.all_parameters[k * 1055 + 658] = value;
	else if(index == 44)
		params.all_parameters[k * 1055 + 665] = value;
	else if(index == 45)
		params.all_parameters[k * 1055 + 672] = value;
	else if(index == 46)
		params.all_parameters[k * 1055 + 679] = value;
	else if(index == 47)
		params.all_parameters[k * 1055 + 686] = value;
	else if(index == 48)
		params.all_parameters[k * 1055 + 693] = value;
	else if(index == 49)
		params.all_parameters[k * 1055 + 700] = value;
	else if(index == 50)
		params.all_parameters[k * 1055 + 707] = value;
	else if(index == 51)
		params.all_parameters[k * 1055 + 714] = value;
	else if(index == 52)
		params.all_parameters[k * 1055 + 721] = value;
	else if(index == 53)
		params.all_parameters[k * 1055 + 728] = value;
	else if(index == 54)
		params.all_parameters[k * 1055 + 735] = value;
	else if(index == 55)
		params.all_parameters[k * 1055 + 742] = value;
	else if(index == 56)
		params.all_parameters[k * 1055 + 749] = value;
	else if(index == 57)
		params.all_parameters[k * 1055 + 756] = value;
	else if(index == 58)
		params.all_parameters[k * 1055 + 763] = value;
	else if(index == 59)
		params.all_parameters[k * 1055 + 770] = value;
	else if(index == 60)
		params.all_parameters[k * 1055 + 777] = value;
	else if(index == 61)
		params.all_parameters[k * 1055 + 784] = value;
	else if(index == 62)
		params.all_parameters[k * 1055 + 791] = value;
	else if(index == 63)
		params.all_parameters[k * 1055 + 798] = value;
	else if(index == 64)
		params.all_parameters[k * 1055 + 805] = value;
	else if(index == 65)
		params.all_parameters[k * 1055 + 812] = value;
	else if(index == 66)
		params.all_parameters[k * 1055 + 819] = value;
	else if(index == 67)
		params.all_parameters[k * 1055 + 826] = value;
	else if(index == 68)
		params.all_parameters[k * 1055 + 833] = value;
	else if(index == 69)
		params.all_parameters[k * 1055 + 840] = value;
	else if(index == 70)
		params.all_parameters[k * 1055 + 847] = value;
	else if(index == 71)
		params.all_parameters[k * 1055 + 854] = value;
	else if(index == 72)
		params.all_parameters[k * 1055 + 861] = value;
	else if(index == 73)
		params.all_parameters[k * 1055 + 868] = value;
	else if(index == 74)
		params.all_parameters[k * 1055 + 875] = value;
	else if(index == 75)
		params.all_parameters[k * 1055 + 882] = value;
	else if(index == 76)
		params.all_parameters[k * 1055 + 889] = value;
	else if(index == 77)
		params.all_parameters[k * 1055 + 896] = value;
	else if(index == 78)
		params.all_parameters[k * 1055 + 903] = value;
	else if(index == 79)
		params.all_parameters[k * 1055 + 910] = value;
	else if(index == 80)
		params.all_parameters[k * 1055 + 917] = value;
	else if(index == 81)
		params.all_parameters[k * 1055 + 924] = value;
	else if(index == 82)
		params.all_parameters[k * 1055 + 931] = value;
	else if(index == 83)
		params.all_parameters[k * 1055 + 938] = value;
	else if(index == 84)
		params.all_parameters[k * 1055 + 945] = value;
	else if(index == 85)
		params.all_parameters[k * 1055 + 952] = value;
	else if(index == 86)
		params.all_parameters[k * 1055 + 959] = value;
	else if(index == 87)
		params.all_parameters[k * 1055 + 966] = value;
	else if(index == 88)
		params.all_parameters[k * 1055 + 973] = value;
	else if(index == 89)
		params.all_parameters[k * 1055 + 980] = value;
	else if(index == 90)
		params.all_parameters[k * 1055 + 987] = value;
	else if(index == 91)
		params.all_parameters[k * 1055 + 994] = value;
	else if(index == 92)
		params.all_parameters[k * 1055 + 1001] = value;
	else if(index == 93)
		params.all_parameters[k * 1055 + 1008] = value;
	else if(index == 94)
		params.all_parameters[k * 1055 + 1015] = value;
	else if(index == 95)
		params.all_parameters[k * 1055 + 1022] = value;
	else if(index == 96)
		params.all_parameters[k * 1055 + 1029] = value;
	else if(index == 97)
		params.all_parameters[k * 1055 + 1036] = value;
	else if(index == 98)
		params.all_parameters[k * 1055 + 1043] = value;
	else if(index == 99)
		params.all_parameters[k * 1055 + 1050] = value;
}
void setSolverParameterEllipsoidObstMajor(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 358] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 365] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 372] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 379] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 386] = value;
	else if(index == 5)
		params.all_parameters[k * 1055 + 393] = value;
	else if(index == 6)
		params.all_parameters[k * 1055 + 400] = value;
	else if(index == 7)
		params.all_parameters[k * 1055 + 407] = value;
	else if(index == 8)
		params.all_parameters[k * 1055 + 414] = value;
	else if(index == 9)
		params.all_parameters[k * 1055 + 421] = value;
	else if(index == 10)
		params.all_parameters[k * 1055 + 428] = value;
	else if(index == 11)
		params.all_parameters[k * 1055 + 435] = value;
	else if(index == 12)
		params.all_parameters[k * 1055 + 442] = value;
	else if(index == 13)
		params.all_parameters[k * 1055 + 449] = value;
	else if(index == 14)
		params.all_parameters[k * 1055 + 456] = value;
	else if(index == 15)
		params.all_parameters[k * 1055 + 463] = value;
	else if(index == 16)
		params.all_parameters[k * 1055 + 470] = value;
	else if(index == 17)
		params.all_parameters[k * 1055 + 477] = value;
	else if(index == 18)
		params.all_parameters[k * 1055 + 484] = value;
	else if(index == 19)
		params.all_parameters[k * 1055 + 491] = value;
	else if(index == 20)
		params.all_parameters[k * 1055 + 498] = value;
	else if(index == 21)
		params.all_parameters[k * 1055 + 505] = value;
	else if(index == 22)
		params.all_parameters[k * 1055 + 512] = value;
	else if(index == 23)
		params.all_parameters[k * 1055 + 519] = value;
	else if(index == 24)
		params.all_parameters[k * 1055 + 526] = value;
	else if(index == 25)
		params.all_parameters[k * 1055 + 533] = value;
	else if(index == 26)
		params.all_parameters[k * 1055 + 540] = value;
	else if(index == 27)
		params.all_parameters[k * 1055 + 547] = value;
	else if(index == 28)
		params.all_parameters[k * 1055 + 554] = value;
	else if(index == 29)
		params.all_parameters[k * 1055 + 561] = value;
	else if(index == 30)
		params.all_parameters[k * 1055 + 568] = value;
	else if(index == 31)
		params.all_parameters[k * 1055 + 575] = value;
	else if(index == 32)
		params.all_parameters[k * 1055 + 582] = value;
	else if(index == 33)
		params.all_parameters[k * 1055 + 589] = value;
	else if(index == 34)
		params.all_parameters[k * 1055 + 596] = value;
	else if(index == 35)
		params.all_parameters[k * 1055 + 603] = value;
	else if(index == 36)
		params.all_parameters[k * 1055 + 610] = value;
	else if(index == 37)
		params.all_parameters[k * 1055 + 617] = value;
	else if(index == 38)
		params.all_parameters[k * 1055 + 624] = value;
	else if(index == 39)
		params.all_parameters[k * 1055 + 631] = value;
	else if(index == 40)
		params.all_parameters[k * 1055 + 638] = value;
	else if(index == 41)
		params.all_parameters[k * 1055 + 645] = value;
	else if(index == 42)
		params.all_parameters[k * 1055 + 652] = value;
	else if(index == 43)
		params.all_parameters[k * 1055 + 659] = value;
	else if(index == 44)
		params.all_parameters[k * 1055 + 666] = value;
	else if(index == 45)
		params.all_parameters[k * 1055 + 673] = value;
	else if(index == 46)
		params.all_parameters[k * 1055 + 680] = value;
	else if(index == 47)
		params.all_parameters[k * 1055 + 687] = value;
	else if(index == 48)
		params.all_parameters[k * 1055 + 694] = value;
	else if(index == 49)
		params.all_parameters[k * 1055 + 701] = value;
	else if(index == 50)
		params.all_parameters[k * 1055 + 708] = value;
	else if(index == 51)
		params.all_parameters[k * 1055 + 715] = value;
	else if(index == 52)
		params.all_parameters[k * 1055 + 722] = value;
	else if(index == 53)
		params.all_parameters[k * 1055 + 729] = value;
	else if(index == 54)
		params.all_parameters[k * 1055 + 736] = value;
	else if(index == 55)
		params.all_parameters[k * 1055 + 743] = value;
	else if(index == 56)
		params.all_parameters[k * 1055 + 750] = value;
	else if(index == 57)
		params.all_parameters[k * 1055 + 757] = value;
	else if(index == 58)
		params.all_parameters[k * 1055 + 764] = value;
	else if(index == 59)
		params.all_parameters[k * 1055 + 771] = value;
	else if(index == 60)
		params.all_parameters[k * 1055 + 778] = value;
	else if(index == 61)
		params.all_parameters[k * 1055 + 785] = value;
	else if(index == 62)
		params.all_parameters[k * 1055 + 792] = value;
	else if(index == 63)
		params.all_parameters[k * 1055 + 799] = value;
	else if(index == 64)
		params.all_parameters[k * 1055 + 806] = value;
	else if(index == 65)
		params.all_parameters[k * 1055 + 813] = value;
	else if(index == 66)
		params.all_parameters[k * 1055 + 820] = value;
	else if(index == 67)
		params.all_parameters[k * 1055 + 827] = value;
	else if(index == 68)
		params.all_parameters[k * 1055 + 834] = value;
	else if(index == 69)
		params.all_parameters[k * 1055 + 841] = value;
	else if(index == 70)
		params.all_parameters[k * 1055 + 848] = value;
	else if(index == 71)
		params.all_parameters[k * 1055 + 855] = value;
	else if(index == 72)
		params.all_parameters[k * 1055 + 862] = value;
	else if(index == 73)
		params.all_parameters[k * 1055 + 869] = value;
	else if(index == 74)
		params.all_parameters[k * 1055 + 876] = value;
	else if(index == 75)
		params.all_parameters[k * 1055 + 883] = value;
	else if(index == 76)
		params.all_parameters[k * 1055 + 890] = value;
	else if(index == 77)
		params.all_parameters[k * 1055 + 897] = value;
	else if(index == 78)
		params.all_parameters[k * 1055 + 904] = value;
	else if(index == 79)
		params.all_parameters[k * 1055 + 911] = value;
	else if(index == 80)
		params.all_parameters[k * 1055 + 918] = value;
	else if(index == 81)
		params.all_parameters[k * 1055 + 925] = value;
	else if(index == 82)
		params.all_parameters[k * 1055 + 932] = value;
	else if(index == 83)
		params.all_parameters[k * 1055 + 939] = value;
	else if(index == 84)
		params.all_parameters[k * 1055 + 946] = value;
	else if(index == 85)
		params.all_parameters[k * 1055 + 953] = value;
	else if(index == 86)
		params.all_parameters[k * 1055 + 960] = value;
	else if(index == 87)
		params.all_parameters[k * 1055 + 967] = value;
	else if(index == 88)
		params.all_parameters[k * 1055 + 974] = value;
	else if(index == 89)
		params.all_parameters[k * 1055 + 981] = value;
	else if(index == 90)
		params.all_parameters[k * 1055 + 988] = value;
	else if(index == 91)
		params.all_parameters[k * 1055 + 995] = value;
	else if(index == 92)
		params.all_parameters[k * 1055 + 1002] = value;
	else if(index == 93)
		params.all_parameters[k * 1055 + 1009] = value;
	else if(index == 94)
		params.all_parameters[k * 1055 + 1016] = value;
	else if(index == 95)
		params.all_parameters[k * 1055 + 1023] = value;
	else if(index == 96)
		params.all_parameters[k * 1055 + 1030] = value;
	else if(index == 97)
		params.all_parameters[k * 1055 + 1037] = value;
	else if(index == 98)
		params.all_parameters[k * 1055 + 1044] = value;
	else if(index == 99)
		params.all_parameters[k * 1055 + 1051] = value;
}
void setSolverParameterEllipsoidObstMinor(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 359] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 366] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 373] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 380] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 387] = value;
	else if(index == 5)
		params.all_parameters[k * 1055 + 394] = value;
	else if(index == 6)
		params.all_parameters[k * 1055 + 401] = value;
	else if(index == 7)
		params.all_parameters[k * 1055 + 408] = value;
	else if(index == 8)
		params.all_parameters[k * 1055 + 415] = value;
	else if(index == 9)
		params.all_parameters[k * 1055 + 422] = value;
	else if(index == 10)
		params.all_parameters[k * 1055 + 429] = value;
	else if(index == 11)
		params.all_parameters[k * 1055 + 436] = value;
	else if(index == 12)
		params.all_parameters[k * 1055 + 443] = value;
	else if(index == 13)
		params.all_parameters[k * 1055 + 450] = value;
	else if(index == 14)
		params.all_parameters[k * 1055 + 457] = value;
	else if(index == 15)
		params.all_parameters[k * 1055 + 464] = value;
	else if(index == 16)
		params.all_parameters[k * 1055 + 471] = value;
	else if(index == 17)
		params.all_parameters[k * 1055 + 478] = value;
	else if(index == 18)
		params.all_parameters[k * 1055 + 485] = value;
	else if(index == 19)
		params.all_parameters[k * 1055 + 492] = value;
	else if(index == 20)
		params.all_parameters[k * 1055 + 499] = value;
	else if(index == 21)
		params.all_parameters[k * 1055 + 506] = value;
	else if(index == 22)
		params.all_parameters[k * 1055 + 513] = value;
	else if(index == 23)
		params.all_parameters[k * 1055 + 520] = value;
	else if(index == 24)
		params.all_parameters[k * 1055 + 527] = value;
	else if(index == 25)
		params.all_parameters[k * 1055 + 534] = value;
	else if(index == 26)
		params.all_parameters[k * 1055 + 541] = value;
	else if(index == 27)
		params.all_parameters[k * 1055 + 548] = value;
	else if(index == 28)
		params.all_parameters[k * 1055 + 555] = value;
	else if(index == 29)
		params.all_parameters[k * 1055 + 562] = value;
	else if(index == 30)
		params.all_parameters[k * 1055 + 569] = value;
	else if(index == 31)
		params.all_parameters[k * 1055 + 576] = value;
	else if(index == 32)
		params.all_parameters[k * 1055 + 583] = value;
	else if(index == 33)
		params.all_parameters[k * 1055 + 590] = value;
	else if(index == 34)
		params.all_parameters[k * 1055 + 597] = value;
	else if(index == 35)
		params.all_parameters[k * 1055 + 604] = value;
	else if(index == 36)
		params.all_parameters[k * 1055 + 611] = value;
	else if(index == 37)
		params.all_parameters[k * 1055 + 618] = value;
	else if(index == 38)
		params.all_parameters[k * 1055 + 625] = value;
	else if(index == 39)
		params.all_parameters[k * 1055 + 632] = value;
	else if(index == 40)
		params.all_parameters[k * 1055 + 639] = value;
	else if(index == 41)
		params.all_parameters[k * 1055 + 646] = value;
	else if(index == 42)
		params.all_parameters[k * 1055 + 653] = value;
	else if(index == 43)
		params.all_parameters[k * 1055 + 660] = value;
	else if(index == 44)
		params.all_parameters[k * 1055 + 667] = value;
	else if(index == 45)
		params.all_parameters[k * 1055 + 674] = value;
	else if(index == 46)
		params.all_parameters[k * 1055 + 681] = value;
	else if(index == 47)
		params.all_parameters[k * 1055 + 688] = value;
	else if(index == 48)
		params.all_parameters[k * 1055 + 695] = value;
	else if(index == 49)
		params.all_parameters[k * 1055 + 702] = value;
	else if(index == 50)
		params.all_parameters[k * 1055 + 709] = value;
	else if(index == 51)
		params.all_parameters[k * 1055 + 716] = value;
	else if(index == 52)
		params.all_parameters[k * 1055 + 723] = value;
	else if(index == 53)
		params.all_parameters[k * 1055 + 730] = value;
	else if(index == 54)
		params.all_parameters[k * 1055 + 737] = value;
	else if(index == 55)
		params.all_parameters[k * 1055 + 744] = value;
	else if(index == 56)
		params.all_parameters[k * 1055 + 751] = value;
	else if(index == 57)
		params.all_parameters[k * 1055 + 758] = value;
	else if(index == 58)
		params.all_parameters[k * 1055 + 765] = value;
	else if(index == 59)
		params.all_parameters[k * 1055 + 772] = value;
	else if(index == 60)
		params.all_parameters[k * 1055 + 779] = value;
	else if(index == 61)
		params.all_parameters[k * 1055 + 786] = value;
	else if(index == 62)
		params.all_parameters[k * 1055 + 793] = value;
	else if(index == 63)
		params.all_parameters[k * 1055 + 800] = value;
	else if(index == 64)
		params.all_parameters[k * 1055 + 807] = value;
	else if(index == 65)
		params.all_parameters[k * 1055 + 814] = value;
	else if(index == 66)
		params.all_parameters[k * 1055 + 821] = value;
	else if(index == 67)
		params.all_parameters[k * 1055 + 828] = value;
	else if(index == 68)
		params.all_parameters[k * 1055 + 835] = value;
	else if(index == 69)
		params.all_parameters[k * 1055 + 842] = value;
	else if(index == 70)
		params.all_parameters[k * 1055 + 849] = value;
	else if(index == 71)
		params.all_parameters[k * 1055 + 856] = value;
	else if(index == 72)
		params.all_parameters[k * 1055 + 863] = value;
	else if(index == 73)
		params.all_parameters[k * 1055 + 870] = value;
	else if(index == 74)
		params.all_parameters[k * 1055 + 877] = value;
	else if(index == 75)
		params.all_parameters[k * 1055 + 884] = value;
	else if(index == 76)
		params.all_parameters[k * 1055 + 891] = value;
	else if(index == 77)
		params.all_parameters[k * 1055 + 898] = value;
	else if(index == 78)
		params.all_parameters[k * 1055 + 905] = value;
	else if(index == 79)
		params.all_parameters[k * 1055 + 912] = value;
	else if(index == 80)
		params.all_parameters[k * 1055 + 919] = value;
	else if(index == 81)
		params.all_parameters[k * 1055 + 926] = value;
	else if(index == 82)
		params.all_parameters[k * 1055 + 933] = value;
	else if(index == 83)
		params.all_parameters[k * 1055 + 940] = value;
	else if(index == 84)
		params.all_parameters[k * 1055 + 947] = value;
	else if(index == 85)
		params.all_parameters[k * 1055 + 954] = value;
	else if(index == 86)
		params.all_parameters[k * 1055 + 961] = value;
	else if(index == 87)
		params.all_parameters[k * 1055 + 968] = value;
	else if(index == 88)
		params.all_parameters[k * 1055 + 975] = value;
	else if(index == 89)
		params.all_parameters[k * 1055 + 982] = value;
	else if(index == 90)
		params.all_parameters[k * 1055 + 989] = value;
	else if(index == 91)
		params.all_parameters[k * 1055 + 996] = value;
	else if(index == 92)
		params.all_parameters[k * 1055 + 1003] = value;
	else if(index == 93)
		params.all_parameters[k * 1055 + 1010] = value;
	else if(index == 94)
		params.all_parameters[k * 1055 + 1017] = value;
	else if(index == 95)
		params.all_parameters[k * 1055 + 1024] = value;
	else if(index == 96)
		params.all_parameters[k * 1055 + 1031] = value;
	else if(index == 97)
		params.all_parameters[k * 1055 + 1038] = value;
	else if(index == 98)
		params.all_parameters[k * 1055 + 1045] = value;
	else if(index == 99)
		params.all_parameters[k * 1055 + 1052] = value;
}
void setSolverParameterEllipsoidObstChi(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 360] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 367] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 374] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 381] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 388] = value;
	else if(index == 5)
		params.all_parameters[k * 1055 + 395] = value;
	else if(index == 6)
		params.all_parameters[k * 1055 + 402] = value;
	else if(index == 7)
		params.all_parameters[k * 1055 + 409] = value;
	else if(index == 8)
		params.all_parameters[k * 1055 + 416] = value;
	else if(index == 9)
		params.all_parameters[k * 1055 + 423] = value;
	else if(index == 10)
		params.all_parameters[k * 1055 + 430] = value;
	else if(index == 11)
		params.all_parameters[k * 1055 + 437] = value;
	else if(index == 12)
		params.all_parameters[k * 1055 + 444] = value;
	else if(index == 13)
		params.all_parameters[k * 1055 + 451] = value;
	else if(index == 14)
		params.all_parameters[k * 1055 + 458] = value;
	else if(index == 15)
		params.all_parameters[k * 1055 + 465] = value;
	else if(index == 16)
		params.all_parameters[k * 1055 + 472] = value;
	else if(index == 17)
		params.all_parameters[k * 1055 + 479] = value;
	else if(index == 18)
		params.all_parameters[k * 1055 + 486] = value;
	else if(index == 19)
		params.all_parameters[k * 1055 + 493] = value;
	else if(index == 20)
		params.all_parameters[k * 1055 + 500] = value;
	else if(index == 21)
		params.all_parameters[k * 1055 + 507] = value;
	else if(index == 22)
		params.all_parameters[k * 1055 + 514] = value;
	else if(index == 23)
		params.all_parameters[k * 1055 + 521] = value;
	else if(index == 24)
		params.all_parameters[k * 1055 + 528] = value;
	else if(index == 25)
		params.all_parameters[k * 1055 + 535] = value;
	else if(index == 26)
		params.all_parameters[k * 1055 + 542] = value;
	else if(index == 27)
		params.all_parameters[k * 1055 + 549] = value;
	else if(index == 28)
		params.all_parameters[k * 1055 + 556] = value;
	else if(index == 29)
		params.all_parameters[k * 1055 + 563] = value;
	else if(index == 30)
		params.all_parameters[k * 1055 + 570] = value;
	else if(index == 31)
		params.all_parameters[k * 1055 + 577] = value;
	else if(index == 32)
		params.all_parameters[k * 1055 + 584] = value;
	else if(index == 33)
		params.all_parameters[k * 1055 + 591] = value;
	else if(index == 34)
		params.all_parameters[k * 1055 + 598] = value;
	else if(index == 35)
		params.all_parameters[k * 1055 + 605] = value;
	else if(index == 36)
		params.all_parameters[k * 1055 + 612] = value;
	else if(index == 37)
		params.all_parameters[k * 1055 + 619] = value;
	else if(index == 38)
		params.all_parameters[k * 1055 + 626] = value;
	else if(index == 39)
		params.all_parameters[k * 1055 + 633] = value;
	else if(index == 40)
		params.all_parameters[k * 1055 + 640] = value;
	else if(index == 41)
		params.all_parameters[k * 1055 + 647] = value;
	else if(index == 42)
		params.all_parameters[k * 1055 + 654] = value;
	else if(index == 43)
		params.all_parameters[k * 1055 + 661] = value;
	else if(index == 44)
		params.all_parameters[k * 1055 + 668] = value;
	else if(index == 45)
		params.all_parameters[k * 1055 + 675] = value;
	else if(index == 46)
		params.all_parameters[k * 1055 + 682] = value;
	else if(index == 47)
		params.all_parameters[k * 1055 + 689] = value;
	else if(index == 48)
		params.all_parameters[k * 1055 + 696] = value;
	else if(index == 49)
		params.all_parameters[k * 1055 + 703] = value;
	else if(index == 50)
		params.all_parameters[k * 1055 + 710] = value;
	else if(index == 51)
		params.all_parameters[k * 1055 + 717] = value;
	else if(index == 52)
		params.all_parameters[k * 1055 + 724] = value;
	else if(index == 53)
		params.all_parameters[k * 1055 + 731] = value;
	else if(index == 54)
		params.all_parameters[k * 1055 + 738] = value;
	else if(index == 55)
		params.all_parameters[k * 1055 + 745] = value;
	else if(index == 56)
		params.all_parameters[k * 1055 + 752] = value;
	else if(index == 57)
		params.all_parameters[k * 1055 + 759] = value;
	else if(index == 58)
		params.all_parameters[k * 1055 + 766] = value;
	else if(index == 59)
		params.all_parameters[k * 1055 + 773] = value;
	else if(index == 60)
		params.all_parameters[k * 1055 + 780] = value;
	else if(index == 61)
		params.all_parameters[k * 1055 + 787] = value;
	else if(index == 62)
		params.all_parameters[k * 1055 + 794] = value;
	else if(index == 63)
		params.all_parameters[k * 1055 + 801] = value;
	else if(index == 64)
		params.all_parameters[k * 1055 + 808] = value;
	else if(index == 65)
		params.all_parameters[k * 1055 + 815] = value;
	else if(index == 66)
		params.all_parameters[k * 1055 + 822] = value;
	else if(index == 67)
		params.all_parameters[k * 1055 + 829] = value;
	else if(index == 68)
		params.all_parameters[k * 1055 + 836] = value;
	else if(index == 69)
		params.all_parameters[k * 1055 + 843] = value;
	else if(index == 70)
		params.all_parameters[k * 1055 + 850] = value;
	else if(index == 71)
		params.all_parameters[k * 1055 + 857] = value;
	else if(index == 72)
		params.all_parameters[k * 1055 + 864] = value;
	else if(index == 73)
		params.all_parameters[k * 1055 + 871] = value;
	else if(index == 74)
		params.all_parameters[k * 1055 + 878] = value;
	else if(index == 75)
		params.all_parameters[k * 1055 + 885] = value;
	else if(index == 76)
		params.all_parameters[k * 1055 + 892] = value;
	else if(index == 77)
		params.all_parameters[k * 1055 + 899] = value;
	else if(index == 78)
		params.all_parameters[k * 1055 + 906] = value;
	else if(index == 79)
		params.all_parameters[k * 1055 + 913] = value;
	else if(index == 80)
		params.all_parameters[k * 1055 + 920] = value;
	else if(index == 81)
		params.all_parameters[k * 1055 + 927] = value;
	else if(index == 82)
		params.all_parameters[k * 1055 + 934] = value;
	else if(index == 83)
		params.all_parameters[k * 1055 + 941] = value;
	else if(index == 84)
		params.all_parameters[k * 1055 + 948] = value;
	else if(index == 85)
		params.all_parameters[k * 1055 + 955] = value;
	else if(index == 86)
		params.all_parameters[k * 1055 + 962] = value;
	else if(index == 87)
		params.all_parameters[k * 1055 + 969] = value;
	else if(index == 88)
		params.all_parameters[k * 1055 + 976] = value;
	else if(index == 89)
		params.all_parameters[k * 1055 + 983] = value;
	else if(index == 90)
		params.all_parameters[k * 1055 + 990] = value;
	else if(index == 91)
		params.all_parameters[k * 1055 + 997] = value;
	else if(index == 92)
		params.all_parameters[k * 1055 + 1004] = value;
	else if(index == 93)
		params.all_parameters[k * 1055 + 1011] = value;
	else if(index == 94)
		params.all_parameters[k * 1055 + 1018] = value;
	else if(index == 95)
		params.all_parameters[k * 1055 + 1025] = value;
	else if(index == 96)
		params.all_parameters[k * 1055 + 1032] = value;
	else if(index == 97)
		params.all_parameters[k * 1055 + 1039] = value;
	else if(index == 98)
		params.all_parameters[k * 1055 + 1046] = value;
	else if(index == 99)
		params.all_parameters[k * 1055 + 1053] = value;
}
void setSolverParameterEllipsoidObstR(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 1055 + 361] = value;
	else if(index == 1)
		params.all_parameters[k * 1055 + 368] = value;
	else if(index == 2)
		params.all_parameters[k * 1055 + 375] = value;
	else if(index == 3)
		params.all_parameters[k * 1055 + 382] = value;
	else if(index == 4)
		params.all_parameters[k * 1055 + 389] = value;
	else if(index == 5)
		params.all_parameters[k * 1055 + 396] = value;
	else if(index == 6)
		params.all_parameters[k * 1055 + 403] = value;
	else if(index == 7)
		params.all_parameters[k * 1055 + 410] = value;
	else if(index == 8)
		params.all_parameters[k * 1055 + 417] = value;
	else if(index == 9)
		params.all_parameters[k * 1055 + 424] = value;
	else if(index == 10)
		params.all_parameters[k * 1055 + 431] = value;
	else if(index == 11)
		params.all_parameters[k * 1055 + 438] = value;
	else if(index == 12)
		params.all_parameters[k * 1055 + 445] = value;
	else if(index == 13)
		params.all_parameters[k * 1055 + 452] = value;
	else if(index == 14)
		params.all_parameters[k * 1055 + 459] = value;
	else if(index == 15)
		params.all_parameters[k * 1055 + 466] = value;
	else if(index == 16)
		params.all_parameters[k * 1055 + 473] = value;
	else if(index == 17)
		params.all_parameters[k * 1055 + 480] = value;
	else if(index == 18)
		params.all_parameters[k * 1055 + 487] = value;
	else if(index == 19)
		params.all_parameters[k * 1055 + 494] = value;
	else if(index == 20)
		params.all_parameters[k * 1055 + 501] = value;
	else if(index == 21)
		params.all_parameters[k * 1055 + 508] = value;
	else if(index == 22)
		params.all_parameters[k * 1055 + 515] = value;
	else if(index == 23)
		params.all_parameters[k * 1055 + 522] = value;
	else if(index == 24)
		params.all_parameters[k * 1055 + 529] = value;
	else if(index == 25)
		params.all_parameters[k * 1055 + 536] = value;
	else if(index == 26)
		params.all_parameters[k * 1055 + 543] = value;
	else if(index == 27)
		params.all_parameters[k * 1055 + 550] = value;
	else if(index == 28)
		params.all_parameters[k * 1055 + 557] = value;
	else if(index == 29)
		params.all_parameters[k * 1055 + 564] = value;
	else if(index == 30)
		params.all_parameters[k * 1055 + 571] = value;
	else if(index == 31)
		params.all_parameters[k * 1055 + 578] = value;
	else if(index == 32)
		params.all_parameters[k * 1055 + 585] = value;
	else if(index == 33)
		params.all_parameters[k * 1055 + 592] = value;
	else if(index == 34)
		params.all_parameters[k * 1055 + 599] = value;
	else if(index == 35)
		params.all_parameters[k * 1055 + 606] = value;
	else if(index == 36)
		params.all_parameters[k * 1055 + 613] = value;
	else if(index == 37)
		params.all_parameters[k * 1055 + 620] = value;
	else if(index == 38)
		params.all_parameters[k * 1055 + 627] = value;
	else if(index == 39)
		params.all_parameters[k * 1055 + 634] = value;
	else if(index == 40)
		params.all_parameters[k * 1055 + 641] = value;
	else if(index == 41)
		params.all_parameters[k * 1055 + 648] = value;
	else if(index == 42)
		params.all_parameters[k * 1055 + 655] = value;
	else if(index == 43)
		params.all_parameters[k * 1055 + 662] = value;
	else if(index == 44)
		params.all_parameters[k * 1055 + 669] = value;
	else if(index == 45)
		params.all_parameters[k * 1055 + 676] = value;
	else if(index == 46)
		params.all_parameters[k * 1055 + 683] = value;
	else if(index == 47)
		params.all_parameters[k * 1055 + 690] = value;
	else if(index == 48)
		params.all_parameters[k * 1055 + 697] = value;
	else if(index == 49)
		params.all_parameters[k * 1055 + 704] = value;
	else if(index == 50)
		params.all_parameters[k * 1055 + 711] = value;
	else if(index == 51)
		params.all_parameters[k * 1055 + 718] = value;
	else if(index == 52)
		params.all_parameters[k * 1055 + 725] = value;
	else if(index == 53)
		params.all_parameters[k * 1055 + 732] = value;
	else if(index == 54)
		params.all_parameters[k * 1055 + 739] = value;
	else if(index == 55)
		params.all_parameters[k * 1055 + 746] = value;
	else if(index == 56)
		params.all_parameters[k * 1055 + 753] = value;
	else if(index == 57)
		params.all_parameters[k * 1055 + 760] = value;
	else if(index == 58)
		params.all_parameters[k * 1055 + 767] = value;
	else if(index == 59)
		params.all_parameters[k * 1055 + 774] = value;
	else if(index == 60)
		params.all_parameters[k * 1055 + 781] = value;
	else if(index == 61)
		params.all_parameters[k * 1055 + 788] = value;
	else if(index == 62)
		params.all_parameters[k * 1055 + 795] = value;
	else if(index == 63)
		params.all_parameters[k * 1055 + 802] = value;
	else if(index == 64)
		params.all_parameters[k * 1055 + 809] = value;
	else if(index == 65)
		params.all_parameters[k * 1055 + 816] = value;
	else if(index == 66)
		params.all_parameters[k * 1055 + 823] = value;
	else if(index == 67)
		params.all_parameters[k * 1055 + 830] = value;
	else if(index == 68)
		params.all_parameters[k * 1055 + 837] = value;
	else if(index == 69)
		params.all_parameters[k * 1055 + 844] = value;
	else if(index == 70)
		params.all_parameters[k * 1055 + 851] = value;
	else if(index == 71)
		params.all_parameters[k * 1055 + 858] = value;
	else if(index == 72)
		params.all_parameters[k * 1055 + 865] = value;
	else if(index == 73)
		params.all_parameters[k * 1055 + 872] = value;
	else if(index == 74)
		params.all_parameters[k * 1055 + 879] = value;
	else if(index == 75)
		params.all_parameters[k * 1055 + 886] = value;
	else if(index == 76)
		params.all_parameters[k * 1055 + 893] = value;
	else if(index == 77)
		params.all_parameters[k * 1055 + 900] = value;
	else if(index == 78)
		params.all_parameters[k * 1055 + 907] = value;
	else if(index == 79)
		params.all_parameters[k * 1055 + 914] = value;
	else if(index == 80)
		params.all_parameters[k * 1055 + 921] = value;
	else if(index == 81)
		params.all_parameters[k * 1055 + 928] = value;
	else if(index == 82)
		params.all_parameters[k * 1055 + 935] = value;
	else if(index == 83)
		params.all_parameters[k * 1055 + 942] = value;
	else if(index == 84)
		params.all_parameters[k * 1055 + 949] = value;
	else if(index == 85)
		params.all_parameters[k * 1055 + 956] = value;
	else if(index == 86)
		params.all_parameters[k * 1055 + 963] = value;
	else if(index == 87)
		params.all_parameters[k * 1055 + 970] = value;
	else if(index == 88)
		params.all_parameters[k * 1055 + 977] = value;
	else if(index == 89)
		params.all_parameters[k * 1055 + 984] = value;
	else if(index == 90)
		params.all_parameters[k * 1055 + 991] = value;
	else if(index == 91)
		params.all_parameters[k * 1055 + 998] = value;
	else if(index == 92)
		params.all_parameters[k * 1055 + 1005] = value;
	else if(index == 93)
		params.all_parameters[k * 1055 + 1012] = value;
	else if(index == 94)
		params.all_parameters[k * 1055 + 1019] = value;
	else if(index == 95)
		params.all_parameters[k * 1055 + 1026] = value;
	else if(index == 96)
		params.all_parameters[k * 1055 + 1033] = value;
	else if(index == 97)
		params.all_parameters[k * 1055 + 1040] = value;
	else if(index == 98)
		params.all_parameters[k * 1055 + 1047] = value;
	else if(index == 99)
		params.all_parameters[k * 1055 + 1054] = value;
}
}
