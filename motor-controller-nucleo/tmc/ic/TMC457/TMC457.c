/*
	This file provides all functions needed for easy
	access to the TMC457 motion control IC.

	Please note that functions for communication over SPI must be added by the user,
	because this is specific to the MCU that is to be used.

	The ReadWriteSPI function with the following parameters and functionality:
	First parameter: indentifies the SPI device
	Second parameter: byte to be sent to the SPI device
	Third parameter: FALSE means that more bytes will follow, so do not relase the
	  chip select line. TRUE means that this was the last byte, so release the chip
	  select line after this byte has been sent and the answer has been fully received.

	The function shall return the byte that has been received via SPI.
*/

#include "TMC457.h"

// Mirror function for the second half of the sine table
#define MIRROR(x) ((x<4096) ? (x) : (8191-x))

// Sine table for initializing the TMC457: sin(2*pi*i/8192)*10000 with i=0..4095
// Instead of the table also a function could be used.
static const u16 IntSinTable[4096]={
		0,    7,   15,   23,   30,   38,   46,   53,   61,   69,   76,   84,   92,   99,  107,  115,
	  122,  130,  138,  145,  153,  161,  168,  176,  184,  191,  199,  207,  214,  222,  230,  237,
	  245,  253,  260,  268,  276,  283,  291,  299,  306,  314,  322,  329,  337,  345,  352,  360,
	  368,  375,  383,  391,  398,  406,  414,  421,  429,  437,  444,  452,  460,  467,  475,  483,
	  490,  498,  506,  513,  521,  529,  536,  544,  552,  559,  567,  574,  582,  590,  597,  605,
	  613,  620,  628,  636,  643,  651,  659,  666,  674,  682,  689,  697,  705,  712,  720,  728,
	  735,  743,  751,  758,  766,  773,  781,  789,  796,  804,  812,  819,  827,  835,  842,  850,
	  858,  865,  873,  881,  888,  896,  903,  911,  919,  926,  934,  942,  949,  957,  965,  972,
	  980,  987,  995, 1003, 1010, 1018, 1026, 1033, 1041, 1048, 1056, 1064, 1071, 1079, 1087, 1094,
	 1102, 1109, 1117, 1125, 1132, 1140, 1148, 1155, 1163, 1170, 1178, 1186, 1193, 1201, 1209, 1216,
	 1224, 1231, 1239, 1247, 1254, 1262, 1269, 1277, 1285, 1292, 1300, 1307, 1315, 1323, 1330, 1338,
	 1345, 1353, 1361, 1368, 1376, 1383, 1391, 1399, 1406, 1414, 1421, 1429, 1437, 1444, 1452, 1459,
	 1467, 1475, 1482, 1490, 1497, 1505, 1512, 1520, 1528, 1535, 1543, 1550, 1558, 1566, 1573, 1581,
	 1588, 1596, 1603, 1611, 1619, 1626, 1634, 1641, 1649, 1656, 1664, 1672, 1679, 1687, 1694, 1702,
	 1709, 1717, 1724, 1732, 1740, 1747, 1755, 1762, 1770, 1777, 1785, 1792, 1800, 1807, 1815, 1823,
	 1830, 1838, 1845, 1853, 1860, 1868, 1875, 1883, 1890, 1898, 1905, 1913, 1921, 1928, 1936, 1943,
	 1951, 1958, 1966, 1973, 1981, 1988, 1996, 2003, 2011, 2018, 2026, 2033, 2041, 2048, 2056, 2063,
	 2071, 2078, 2086, 2093, 2101, 2108, 2116, 2123, 2131, 2138, 2146, 2153, 2161, 2168, 2176, 2183,
	 2191, 2198, 2206, 2213, 2221, 2228, 2236, 2243, 2251, 2258, 2266, 2273, 2280, 2288, 2295, 2303,
	 2310, 2318, 2325, 2333, 2340, 2348, 2355, 2363, 2370, 2377, 2385, 2392, 2400, 2407, 2415, 2422,
	 2430, 2437, 2444, 2452, 2459, 2467, 2474, 2482, 2489, 2497, 2504, 2511, 2519, 2526, 2534, 2541,
	 2548, 2556, 2563, 2571, 2578, 2586, 2593, 2600, 2608, 2615, 2623, 2630, 2637, 2645, 2652, 2660,
	 2667, 2674, 2682, 2689, 2697, 2704, 2711, 2719, 2726, 2733, 2741, 2748, 2756, 2763, 2770, 2778,
	 2785, 2792, 2800, 2807, 2814, 2822, 2829, 2837, 2844, 2851, 2859, 2866, 2873, 2881, 2888, 2895,
	 2903, 2910, 2917, 2925, 2932, 2939, 2947, 2954, 2961, 2969, 2976, 2983, 2991, 2998, 3005, 3013,
	 3020, 3027, 3035, 3042, 3049, 3056, 3064, 3071, 3078, 3086, 3093, 3100, 3108, 3115, 3122, 3129,
	 3137, 3144, 3151, 3159, 3166, 3173, 3180, 3188, 3195, 3202, 3209, 3217, 3224, 3231, 3238, 3246,
	 3253, 3260, 3267, 3275, 3282, 3289, 3296, 3304, 3311, 3318, 3325, 3333, 3340, 3347, 3354, 3362,
	 3369, 3376, 3383, 3390, 3398, 3405, 3412, 3419, 3427, 3434, 3441, 3448, 3455, 3463, 3470, 3477,
	 3484, 3491, 3498, 3506, 3513, 3520, 3527, 3534, 3542, 3549, 3556, 3563, 3570, 3577, 3585, 3592,
	 3599, 3606, 3613, 3620, 3627, 3635, 3642, 3649, 3656, 3663, 3670, 3677, 3685, 3692, 3699, 3706,
	 3713, 3720, 3727, 3734, 3742, 3749, 3756, 3763, 3770, 3777, 3784, 3791, 3798, 3806, 3813, 3820,
	 3827, 3834, 3841, 3848, 3855, 3862, 3869, 3876, 3883, 3890, 3898, 3905, 3912, 3919, 3926, 3933,
	 3940, 3947, 3954, 3961, 3968, 3975, 3982, 3989, 3996, 4003, 4010, 4017, 4024, 4031, 4038, 4045,
	 4052, 4059, 4066, 4073, 4080, 4087, 4094, 4101, 4108, 4115, 4122, 4129, 4136, 4143, 4150, 4157,
	 4164, 4171, 4178, 4185, 4192, 4199, 4206, 4213, 4220, 4227, 4234, 4241, 4248, 4255, 4262, 4269,
	 4276, 4282, 4289, 4296, 4303, 4310, 4317, 4324, 4331, 4338, 4345, 4352, 4359, 4365, 4372, 4379,
	 4386, 4393, 4400, 4407, 4414, 4421, 4427, 4434, 4441, 4448, 4455, 4462, 4469, 4476, 4482, 4489,
	 4496, 4503, 4510, 4517, 4524, 4530, 4537, 4544, 4551, 4558, 4565, 4571, 4578, 4585, 4592, 4599,
	 4605, 4612, 4619, 4626, 4633, 4639, 4646, 4653, 4660, 4667, 4673, 4680, 4687, 4694, 4700, 4707,
	 4714, 4721, 4728, 4734, 4741, 4748, 4755, 4761, 4768, 4775, 4782, 4788, 4795, 4802, 4808, 4815,
	 4822, 4829, 4835, 4842, 4849, 4855, 4862, 4869, 4876, 4882, 4889, 4896, 4902, 4909, 4916, 4922,
	 4929, 4936, 4942, 4949, 4956, 4962, 4969, 4976, 4982, 4989, 4996, 5002, 5009, 5016, 5022, 5029,
	 5035, 5042, 5049, 5055, 5062, 5069, 5075, 5082, 5088, 5095, 5102, 5108, 5115, 5121, 5128, 5135,
	 5141, 5148, 5154, 5161, 5167, 5174, 5181, 5187, 5194, 5200, 5207, 5213, 5220, 5226, 5233, 5239,
	 5246, 5252, 5259, 5266, 5272, 5279, 5285, 5292, 5298, 5305, 5311, 5318, 5324, 5331, 5337, 5344,
	 5350, 5357, 5363, 5369, 5376, 5382, 5389, 5395, 5402, 5408, 5415, 5421, 5428, 5434, 5440, 5447,
	 5453, 5460, 5466, 5473, 5479, 5485, 5492, 5498, 5505, 5511, 5517, 5524, 5530, 5537, 5543, 5549,
	 5556, 5562, 5569, 5575, 5581, 5588, 5594, 5600, 5607, 5613, 5619, 5626, 5632, 5638, 5645, 5651,
	 5657, 5664, 5670, 5676, 5683, 5689, 5695, 5702, 5708, 5714, 5721, 5727, 5733, 5739, 5746, 5752,
	 5758, 5764, 5771, 5777, 5783, 5790, 5796, 5802, 5808, 5814, 5821, 5827, 5833, 5839, 5846, 5852,
	 5858, 5864, 5871, 5877, 5883, 5889, 5895, 5902, 5908, 5914, 5920, 5926, 5932, 5939, 5945, 5951,
	 5957, 5963, 5969, 5976, 5982, 5988, 5994, 6000, 6006, 6012, 6019, 6025, 6031, 6037, 6043, 6049,
	 6055, 6061, 6067, 6074, 6080, 6086, 6092, 6098, 6104, 6110, 6116, 6122, 6128, 6134, 6140, 6146,
	 6152, 6158, 6165, 6171, 6177, 6183, 6189, 6195, 6201, 6207, 6213, 6219, 6225, 6231, 6237, 6243,
	 6249, 6255, 6261, 6267, 6273, 6279, 6285, 6291, 6297, 6302, 6308, 6314, 6320, 6326, 6332, 6338,
	 6344, 6350, 6356, 6362, 6368, 6374, 6380, 6385, 6391, 6397, 6403, 6409, 6415, 6421, 6427, 6433,
	 6438, 6444, 6450, 6456, 6462, 6468, 6474, 6479, 6485, 6491, 6497, 6503, 6509, 6514, 6520, 6526,
	 6532, 6538, 6543, 6549, 6555, 6561, 6567, 6572, 6578, 6584, 6590, 6596, 6601, 6607, 6613, 6619,
	 6624, 6630, 6636, 6642, 6647, 6653, 6659, 6664, 6670, 6676, 6682, 6687, 6693, 6699, 6704, 6710,
	 6716, 6721, 6727, 6733, 6738, 6744, 6750, 6755, 6761, 6767, 6772, 6778, 6784, 6789, 6795, 6801,
	 6806, 6812, 6817, 6823, 6829, 6834, 6840, 6845, 6851, 6857, 6862, 6868, 6873, 6879, 6884, 6890,
	 6896, 6901, 6907, 6912, 6918, 6923, 6929, 6934, 6940, 6945, 6951, 6956, 6962, 6967, 6973, 6978,
	 6984, 6989, 6995, 7000, 7006, 7011, 7017, 7022, 7028, 7033, 7039, 7044, 7050, 7055, 7060, 7066,
	 7071, 7077, 7082, 7087, 7093, 7098, 7104, 7109, 7115, 7120, 7125, 7131, 7136, 7141, 7147, 7152,
	 7157, 7163, 7168, 7174, 7179, 7184, 7190, 7195, 7200, 7206, 7211, 7216, 7221, 7227, 7232, 7237,
	 7243, 7248, 7253, 7258, 7264, 7269, 7274, 7280, 7285, 7290, 7295, 7301, 7306, 7311, 7316, 7322,
	 7327, 7332, 7337, 7342, 7348, 7353, 7358, 7363, 7368, 7374, 7379, 7384, 7389, 7394, 7399, 7405,
	 7410, 7415, 7420, 7425, 7430, 7435, 7441, 7446, 7451, 7456, 7461, 7466, 7471, 7476, 7481, 7486,
	 7492, 7497, 7502, 7507, 7512, 7517, 7522, 7527, 7532, 7537, 7542, 7547, 7552, 7557, 7562, 7567,
	 7572, 7577, 7582, 7587, 7592, 7597, 7602, 7607, 7612, 7617, 7622, 7627, 7632, 7637, 7642, 7647,
	 7652, 7657, 7662, 7667, 7672, 7676, 7681, 7686, 7691, 7696, 7701, 7706, 7711, 7716, 7721, 7725,
	 7730, 7735, 7740, 7745, 7750, 7755, 7759, 7764, 7769, 7774, 7779, 7784, 7788, 7793, 7798, 7803,
	 7808, 7812, 7817, 7822, 7827, 7831, 7836, 7841, 7846, 7851, 7855, 7860, 7865, 7869, 7874, 7879,
	 7884, 7888, 7893, 7898, 7902, 7907, 7912, 7917, 7921, 7926, 7931, 7935, 7940, 7945, 7949, 7954,
	 7959, 7963, 7968, 7972, 7977, 7982, 7986, 7991, 7996, 8000, 8005, 8009, 8014, 8019, 8023, 8028,
	 8032, 8037, 8041, 8046, 8050, 8055, 8060, 8064, 8069, 8073, 8078, 8082, 8087, 8091, 8096, 8100,
	 8105, 8109, 8114, 8118, 8123, 8127, 8132, 8136, 8141, 8145, 8149, 8154, 8158, 8163, 8167, 8172,
	 8176, 8180, 8185, 8189, 8194, 8198, 8202, 8207, 8211, 8216, 8220, 8224, 8229, 8233, 8237, 8242,
	 8246, 8250, 8255, 8259, 8263, 8268, 8272, 8276, 8281, 8285, 8289, 8293, 8298, 8302, 8306, 8311,
	 8315, 8319, 8323, 8328, 8332, 8336, 8340, 8345, 8349, 8353, 8357, 8361, 8366, 8370, 8374, 8378,
	 8382, 8387, 8391, 8395, 8399, 8403, 8407, 8412, 8416, 8420, 8424, 8428, 8432, 8436, 8440, 8445,
	 8449, 8453, 8457, 8461, 8465, 8469, 8473, 8477, 8481, 8485, 8489, 8494, 8498, 8502, 8506, 8510,
	 8514, 8518, 8522, 8526, 8530, 8534, 8538, 8542, 8546, 8550, 8554, 8558, 8562, 8566, 8570, 8573,
	 8577, 8581, 8585, 8589, 8593, 8597, 8601, 8605, 8609, 8613, 8617, 8620, 8624, 8628, 8632, 8636,
	 8640, 8644, 8648, 8651, 8655, 8659, 8663, 8667, 8671, 8674, 8678, 8682, 8686, 8690, 8693, 8697,
	 8701, 8705, 8709, 8712, 8716, 8720, 8724, 8727, 8731, 8735, 8739, 8742, 8746, 8750, 8753, 8757,
	 8761, 8765, 8768, 8772, 8776, 8779, 8783, 8787, 8790, 8794, 8798, 8801, 8805, 8808, 8812, 8816,
	 8819, 8823, 8827, 8830, 8834, 8837, 8841, 8845, 8848, 8852, 8855, 8859, 8862, 8866, 8869, 8873,
	 8877, 8880, 8884, 8887, 8891, 8894, 8898, 8901, 8905, 8908, 8912, 8915, 8919, 8922, 8925, 8929,
	 8932, 8936, 8939, 8943, 8946, 8950, 8953, 8956, 8960, 8963, 8967, 8970, 8973, 8977, 8980, 8983,
	 8987, 8990, 8994, 8997, 9000, 9004, 9007, 9010, 9014, 9017, 9020, 9024, 9027, 9030, 9033, 9037,
	 9040, 9043, 9047, 9050, 9053, 9056, 9060, 9063, 9066, 9069, 9073, 9076, 9079, 9082, 9085, 9089,
	 9092, 9095, 9098, 9101, 9104, 9108, 9111, 9114, 9117, 9120, 9123, 9127, 9130, 9133, 9136, 9139,
	 9142, 9145, 9148, 9151, 9155, 9158, 9161, 9164, 9167, 9170, 9173, 9176, 9179, 9182, 9185, 9188,
	 9191, 9194, 9197, 9200, 9203, 9206, 9209, 9212, 9215, 9218, 9221, 9224, 9227, 9230, 9233, 9236,
	 9239, 9242, 9245, 9248, 9251, 9253, 9256, 9259, 9262, 9265, 9268, 9271, 9274, 9277, 9279, 9282,
	 9285, 9288, 9291, 9294, 9296, 9299, 9302, 9305, 9308, 9311, 9313, 9316, 9319, 9322, 9324, 9327,
	 9330, 9333, 9335, 9338, 9341, 9344, 9346, 9349, 9352, 9355, 9357, 9360, 9363, 9365, 9368, 9371,
	 9373, 9376, 9379, 9381, 9384, 9387, 9389, 9392, 9395, 9397, 9400, 9402, 9405, 9408, 9410, 9413,
	 9415, 9418, 9421, 9423, 9426, 9428, 9431, 9433, 9436, 9438, 9441, 9444, 9446, 9449, 9451, 9454,
	 9456, 9459, 9461, 9464, 9466, 9468, 9471, 9473, 9476, 9478, 9481, 9483, 9486, 9488, 9490, 9493,
	 9495, 9498, 9500, 9502, 9505, 9507, 9510, 9512, 9514, 9517, 9519, 9521, 9524, 9526, 9528, 9531,
	 9533, 9535, 9538, 9540, 9542, 9545, 9547, 9549, 9551, 9554, 9556, 9558, 9560, 9563, 9565, 9567,
	 9569, 9572, 9574, 9576, 9578, 9580, 9583, 9585, 9587, 9589, 9591, 9593, 9596, 9598, 9600, 9602,
	 9604, 9606, 9609, 9611, 9613, 9615, 9617, 9619, 9621, 9623, 9625, 9627, 9629, 9632, 9634, 9636,
	 9638, 9640, 9642, 9644, 9646, 9648, 9650, 9652, 9654, 9656, 9658, 9660, 9662, 9664, 9666, 9668,
	 9670, 9672, 9674, 9676, 9677, 9679, 9681, 9683, 9685, 9687, 9689, 9691, 9693, 9695, 9696, 9698,
	 9700, 9702, 9704, 9706, 9708, 9709, 9711, 9713, 9715, 9717, 9719, 9720, 9722, 9724, 9726, 9728,
	 9729, 9731, 9733, 9735, 9736, 9738, 9740, 9742, 9743, 9745, 9747, 9748, 9750, 9752, 9754, 9755,
	 9757, 9759, 9760, 9762, 9764, 9765, 9767, 9768, 9770, 9772, 9773, 9775, 9777, 9778, 9780, 9781,
	 9783, 9785, 9786, 9788, 9789, 9791, 9792, 9794, 9796, 9797, 9799, 9800, 9802, 9803, 9805, 9806,
	 9808, 9809, 9811, 9812, 9814, 9815, 9817, 9818, 9819, 9821, 9822, 9824, 9825, 9827, 9828, 9829,
	 9831, 9832, 9834, 9835, 9836, 9838, 9839, 9841, 9842, 9843, 9845, 9846, 9847, 9849, 9850, 9851,
	 9853, 9854, 9855, 9856, 9858, 9859, 9860, 9862, 9863, 9864, 9865, 9867, 9868, 9869, 9870, 9872,
	 9873, 9874, 9875, 9876, 9878, 9879, 9880, 9881, 9882, 9884, 9885, 9886, 9887, 9888, 9889, 9890,
	 9892, 9893, 9894, 9895, 9896, 9897, 9898, 9899, 9900, 9901, 9902, 9904, 9905, 9906, 9907, 9908,
	 9909, 9910, 9911, 9912, 9913, 9914, 9915, 9916, 9917, 9918, 9919, 9920, 9921, 9922, 9923, 9924,
	 9925, 9925, 9926, 9927, 9928, 9929, 9930, 9931, 9932, 9933, 9934, 9934, 9935, 9936, 9937, 9938,
	 9939, 9940, 9940, 9941, 9942, 9943, 9944, 9945, 9945, 9946, 9947, 9948, 9948, 9949, 9950, 9951,
	 9952, 9952, 9953, 9954, 9954, 9955, 9956, 9957, 9957, 9958, 9959, 9959, 9960, 9961, 9961, 9962,
	 9963, 9963, 9964, 9965, 9965, 9966, 9967, 9967, 9968, 9968, 9969, 9970, 9970, 9971, 9971, 9972,
	 9973, 9973, 9974, 9974, 9975, 9975, 9976, 9976, 9977, 9977, 9978, 9978, 9979, 9979, 9980, 9980,
	 9981, 9981, 9982, 9982, 9983, 9983, 9984, 9984, 9984, 9985, 9985, 9986, 9986, 9986, 9987, 9987,
	 9988, 9988, 9988, 9989, 9989, 9989, 9990, 9990, 9990, 9991, 9991, 9991, 9992, 9992, 9992, 9993,
	 9993, 9993, 9993, 9994, 9994, 9994, 9994, 9995, 9995, 9995, 9995, 9996, 9996, 9996, 9996, 9996,
	 9997, 9997, 9997, 9997, 9997, 9997, 9998, 9998, 9998, 9998, 9998, 9998, 9998, 9998, 9999, 9999,
	 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999,
	10000, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999,
	 9999, 9999, 9999, 9998, 9998, 9998, 9998, 9998, 9998, 9998, 9997, 9997, 9997, 9997, 9997, 9997,
	 9996, 9996, 9996, 9996, 9996, 9995, 9995, 9995, 9995, 9994, 9994, 9994, 9994, 9993, 9993, 9993,
	 9993, 9992, 9992, 9992, 9991, 9991, 9991, 9991, 9990, 9990, 9990, 9989, 9989, 9988, 9988, 9988,
	 9987, 9987, 9987, 9986, 9986, 9985, 9985, 9985, 9984, 9984, 9983, 9983, 9982, 9982, 9981, 9981,
	 9981, 9980, 9980, 9979, 9979, 9978, 9978, 9977, 9977, 9976, 9976, 9975, 9974, 9974, 9973, 9973,
	 9972, 9972, 9971, 9971, 9970, 9969, 9969, 9968, 9968, 9967, 9966, 9966, 9965, 9964, 9964, 9963,
	 9962, 9962, 9961, 9960, 9960, 9959, 9958, 9958, 9957, 9956, 9956, 9955, 9954, 9953, 9953, 9952,
	 9951, 9950, 9950, 9949, 9948, 9947, 9947, 9946, 9945, 9944, 9943, 9943, 9942, 9941, 9940, 9939,
	 9938, 9937, 9937, 9936, 9935, 9934, 9933, 9932, 9931, 9930, 9930, 9929, 9928, 9927, 9926, 9925,
	 9924, 9923, 9922, 9921, 9920, 9919, 9918, 9917, 9916, 9915, 9914, 9913, 9912, 9911, 9910, 9909,
	 9908, 9907, 9906, 9905, 9904, 9903, 9902, 9901, 9900, 9899, 9898, 9897, 9895, 9894, 9893, 9892,
	 9891, 9890, 9889, 9888, 9886, 9885, 9884, 9883, 9882, 9881, 9879, 9878, 9877, 9876, 9875, 9873,
	 9872, 9871, 9870, 9868, 9867, 9866, 9865, 9863, 9862, 9861, 9860, 9858, 9857, 9856, 9855, 9853,
	 9852, 9851, 9849, 9848, 9847, 9845, 9844, 9843, 9841, 9840, 9838, 9837, 9836, 9834, 9833, 9832,
	 9830, 9829, 9827, 9826, 9824, 9823, 9822, 9820, 9819, 9817, 9816, 9814, 9813, 9811, 9810, 9808,
	 9807, 9805, 9804, 9802, 9801, 9799, 9798, 9796, 9795, 9793, 9792, 9790, 9789, 9787, 9785, 9784,
	 9782, 9781, 9779, 9777, 9776, 9774, 9773, 9771, 9769, 9768, 9766, 9764, 9763, 9761, 9759, 9758,
	 9756, 9754, 9753, 9751, 9749, 9748, 9746, 9744, 9742, 9741, 9739, 9737, 9735, 9734, 9732, 9730,
	 9728, 9727, 9725, 9723, 9721, 9719, 9718, 9716, 9714, 9712, 9710, 9709, 9707, 9705, 9703, 9701,
	 9699, 9697, 9696, 9694, 9692, 9690, 9688, 9686, 9684, 9682, 9680, 9678, 9676, 9675, 9673, 9671,
	 9669, 9667, 9665, 9663, 9661, 9659, 9657, 9655, 9653, 9651, 9649, 9647, 9645, 9643, 9641, 9639,
	 9637, 9635, 9633, 9630, 9628, 9626, 9624, 9622, 9620, 9618, 9616, 9614, 9612, 9610, 9607, 9605,
	 9603, 9601, 9599, 9597, 9595, 9592, 9590, 9588, 9586, 9584, 9582, 9579, 9577, 9575, 9573, 9570,
	 9568, 9566, 9564, 9562, 9559, 9557, 9555, 9553, 9550, 9548, 9546, 9543, 9541, 9539, 9536, 9534,
	 9532, 9530, 9527, 9525, 9523, 9520, 9518, 9516, 9513, 9511, 9508, 9506, 9504, 9501, 9499, 9496,
	 9494, 9492, 9489, 9487, 9484, 9482, 9480, 9477, 9475, 9472, 9470, 9467, 9465, 9462, 9460, 9457,
	 9455, 9452, 9450, 9447, 9445, 9442, 9440, 9437, 9435, 9432, 9430, 9427, 9424, 9422, 9419, 9417,
	 9414, 9412, 9409, 9406, 9404, 9401, 9399, 9396, 9393, 9391, 9388, 9385, 9383, 9380, 9377, 9375,
	 9372, 9369, 9367, 9364, 9361, 9359, 9356, 9353, 9350, 9348, 9345, 9342, 9340, 9337, 9334, 9331,
	 9329, 9326, 9323, 9320, 9317, 9315, 9312, 9309, 9306, 9303, 9301, 9298, 9295, 9292, 9289, 9287,
	 9284, 9281, 9278, 9275, 9272, 9269, 9266, 9264, 9261, 9258, 9255, 9252, 9249, 9246, 9243, 9240,
	 9237, 9234, 9231, 9229, 9226, 9223, 9220, 9217, 9214, 9211, 9208, 9205, 9202, 9199, 9196, 9193,
	 9190, 9187, 9184, 9181, 9178, 9175, 9171, 9168, 9165, 9162, 9159, 9156, 9153, 9150, 9147, 9144,
	 9141, 9138, 9134, 9131, 9128, 9125, 9122, 9119, 9116, 9112, 9109, 9106, 9103, 9100, 9097, 9093,
	 9090, 9087, 9084, 9081, 9077, 9074, 9071, 9068, 9064, 9061, 9058, 9055, 9051, 9048, 9045, 9042,
	 9038, 9035, 9032, 9028, 9025, 9022, 9019, 9015, 9012, 9009, 9005, 9002, 8999, 8995, 8992, 8989,
	 8985, 8982, 8978, 8975, 8972, 8968, 8965, 8961, 8958, 8955, 8951, 8948, 8944, 8941, 8938, 8934,
	 8931, 8927, 8924, 8920, 8917, 8913, 8910, 8906, 8903, 8899, 8896, 8892, 8889, 8885, 8882, 8878,
	 8875, 8871, 8868, 8864, 8861, 8857, 8853, 8850, 8846, 8843, 8839, 8836, 8832, 8828, 8825, 8821,
	 8818, 8814, 8810, 8807, 8803, 8799, 8796, 8792, 8788, 8785, 8781, 8777, 8774, 8770, 8766, 8763,
	 8759, 8755, 8752, 8748, 8744, 8740, 8737, 8733, 8729, 8725, 8722, 8718, 8714, 8710, 8707, 8703,
	 8699, 8695, 8692, 8688, 8684, 8680, 8676, 8673, 8669, 8665, 8661, 8657, 8653, 8650, 8646, 8642,
	 8638, 8634, 8630, 8626, 8622, 8619, 8615, 8611, 8607, 8603, 8599, 8595, 8591, 8587, 8583, 8579,
	 8575, 8572, 8568, 8564, 8560, 8556, 8552, 8548, 8544, 8540, 8536, 8532, 8528, 8524, 8520, 8516,
	 8512, 8508, 8504, 8500, 8496, 8492, 8487, 8483, 8479, 8475, 8471, 8467, 8463, 8459, 8455, 8451,
	 8447, 8443, 8438, 8434, 8430, 8426, 8422, 8418, 8414, 8409, 8405, 8401, 8397, 8393, 8389, 8384,
	 8380, 8376, 8372, 8368, 8364, 8359, 8355, 8351, 8347, 8342, 8338, 8334, 8330, 8325, 8321, 8317,
	 8313, 8308, 8304, 8300, 8296, 8291, 8287, 8283, 8278, 8274, 8270, 8266, 8261, 8257, 8253, 8248,
	 8244, 8240, 8235, 8231, 8226, 8222, 8218, 8213, 8209, 8205, 8200, 8196, 8191, 8187, 8183, 8178,
	 8174, 8169, 8165, 8161, 8156, 8152, 8147, 8143, 8138, 8134, 8129, 8125, 8120, 8116, 8111, 8107,
	 8103, 8098, 8094, 8089, 8084, 8080, 8075, 8071, 8066, 8062, 8057, 8053, 8048, 8044, 8039, 8035,
	 8030, 8025, 8021, 8016, 8012, 8007, 8002, 7998, 7993, 7989, 7984, 7979, 7975, 7970, 7966, 7961,
	 7956, 7952, 7947, 7942, 7938, 7933, 7928, 7924, 7919, 7914, 7910, 7905, 7900, 7895, 7891, 7886,
	 7881, 7877, 7872, 7867, 7862, 7858, 7853, 7848, 7843, 7839, 7834, 7829, 7824, 7820, 7815, 7810,
	 7805, 7800, 7796, 7791, 7786, 7781, 7776, 7771, 7767, 7762, 7757, 7752, 7747, 7742, 7738, 7733,
	 7728, 7723, 7718, 7713, 7708, 7703, 7699, 7694, 7689, 7684, 7679, 7674, 7669, 7664, 7659, 7654,
	 7649, 7644, 7639, 7635, 7630, 7625, 7620, 7615, 7610, 7605, 7600, 7595, 7590, 7585, 7580, 7575,
	 7570, 7565, 7560, 7555, 7550, 7545, 7540, 7535, 7530, 7524, 7519, 7514, 7509, 7504, 7499, 7494,
	 7489, 7484, 7479, 7474, 7469, 7464, 7458, 7453, 7448, 7443, 7438, 7433, 7428, 7423, 7417, 7412,
	 7407, 7402, 7397, 7392, 7386, 7381, 7376, 7371, 7366, 7361, 7355, 7350, 7345, 7340, 7335, 7329,
	 7324, 7319, 7314, 7308, 7303, 7298, 7293, 7287, 7282, 7277, 7272, 7266, 7261, 7256, 7251, 7245,
	 7240, 7235, 7229, 7224, 7219, 7213, 7208, 7203, 7198, 7192, 7187, 7182, 7176, 7171, 7166, 7160,
	 7155, 7149, 7144, 7139, 7133, 7128, 7123, 7117, 7112, 7106, 7101, 7096, 7090, 7085, 7079, 7074,
	 7069, 7063, 7058, 7052, 7047, 7041, 7036, 7030, 7025, 7020, 7014, 7009, 7003, 6998, 6992, 6987,
	 6981, 6976, 6970, 6965, 6959, 6954, 6948, 6943, 6937, 6932, 6926, 6921, 6915, 6909, 6904, 6898,
	 6893, 6887, 6882, 6876, 6871, 6865, 6859, 6854, 6848, 6843, 6837, 6831, 6826, 6820, 6815, 6809,
	 6803, 6798, 6792, 6786, 6781, 6775, 6770, 6764, 6758, 6753, 6747, 6741, 6736, 6730, 6724, 6719,
	 6713, 6707, 6702, 6696, 6690, 6684, 6679, 6673, 6667, 6662, 6656, 6650, 6644, 6639, 6633, 6627,
	 6621, 6616, 6610, 6604, 6598, 6593, 6587, 6581, 6575, 6570, 6564, 6558, 6552, 6546, 6541, 6535,
	 6529, 6523, 6517, 6512, 6506, 6500, 6494, 6488, 6482, 6477, 6471, 6465, 6459, 6453, 6447, 6441,
	 6436, 6430, 6424, 6418, 6412, 6406, 6400, 6394, 6388, 6383, 6377, 6371, 6365, 6359, 6353, 6347,
	 6341, 6335, 6329, 6323, 6317, 6311, 6305, 6300, 6294, 6288, 6282, 6276, 6270, 6264, 6258, 6252,
	 6246, 6240, 6234, 6228, 6222, 6216, 6210, 6204, 6198, 6192, 6186, 6180, 6174, 6168, 6162, 6155,
	 6149, 6143, 6137, 6131, 6125, 6119, 6113, 6107, 6101, 6095, 6089, 6083, 6077, 6070, 6064, 6058,
	 6052, 6046, 6040, 6034, 6028, 6022, 6015, 6009, 6003, 5997, 5991, 5985, 5979, 5973, 5966, 5960,
	 5954, 5948, 5942, 5936, 5929, 5923, 5917, 5911, 5905, 5898, 5892, 5886, 5880, 5874, 5867, 5861,
	 5855, 5849, 5843, 5836, 5830, 5824, 5818, 5811, 5805, 5799, 5793, 5786, 5780, 5774, 5768, 5761,
	 5755, 5749, 5743, 5736, 5730, 5724, 5717, 5711, 5705, 5698, 5692, 5686, 5680, 5673, 5667, 5661,
	 5654, 5648, 5642, 5635, 5629, 5623, 5616, 5610, 5604, 5597, 5591, 5584, 5578, 5572, 5565, 5559,
	 5553, 5546, 5540, 5533, 5527, 5521, 5514, 5508, 5501, 5495, 5489, 5482, 5476, 5469, 5463, 5457,
	 5450, 5444, 5437, 5431, 5424, 5418, 5411, 5405, 5399, 5392, 5386, 5379, 5373, 5366, 5360, 5353,
	 5347, 5340, 5334, 5327, 5321, 5314, 5308, 5301, 5295, 5288, 5282, 5275, 5269, 5262, 5256, 5249,
	 5243, 5236, 5230, 5223, 5217, 5210, 5203, 5197, 5190, 5184, 5177, 5171, 5164, 5158, 5151, 5144,
	 5138, 5131, 5125, 5118, 5111, 5105, 5098, 5092, 5085, 5078, 5072, 5065, 5059, 5052, 5045, 5039,
	 5032, 5025, 5019, 5012, 5006, 4999, 4992, 4986, 4979, 4972, 4966, 4959, 4952, 4946, 4939, 4932,
	 4926, 4919, 4912, 4906, 4899, 4892, 4886, 4879, 4872, 4865, 4859, 4852, 4845, 4839, 4832, 4825,
	 4819, 4812, 4805, 4798, 4792, 4785, 4778, 4771, 4765, 4758, 4751, 4744, 4738, 4731, 4724, 4717,
	 4711, 4704, 4697, 4690, 4684, 4677, 4670, 4663, 4656, 4650, 4643, 4636, 4629, 4622, 4616, 4609,
	 4602, 4595, 4588, 4582, 4575, 4568, 4561, 4554, 4547, 4541, 4534, 4527, 4520, 4513, 4506, 4500,
	 4493, 4486, 4479, 4472, 4465, 4458, 4452, 4445, 4438, 4431, 4424, 4417, 4410, 4403, 4396, 4390,
	 4383, 4376, 4369, 4362, 4355, 4348, 4341, 4334, 4327, 4321, 4314, 4307, 4300, 4293, 4286, 4279,
	 4272, 4265, 4258, 4251, 4244, 4237, 4230, 4223, 4217, 4210, 4203, 4196, 4189, 4182, 4175, 4168,
	 4161, 4154, 4147, 4140, 4133, 4126, 4119, 4112, 4105, 4098, 4091, 4084, 4077, 4070, 4063, 4056,
	 4049, 4042, 4035, 4028, 4021, 4014, 4007, 4000, 3993, 3986, 3979, 3972, 3965, 3957, 3950, 3943,
	 3936, 3929, 3922, 3915, 3908, 3901, 3894, 3887, 3880, 3873, 3866, 3859, 3852, 3844, 3837, 3830,
	 3823, 3816, 3809, 3802, 3795, 3788, 3781, 3774, 3766, 3759, 3752, 3745, 3738, 3731, 3724, 3717,
	 3710, 3702, 3695, 3688, 3681, 3674, 3667, 3660, 3652, 3645, 3638, 3631, 3624, 3617, 3610, 3602,
	 3595, 3588, 3581, 3574, 3567, 3559, 3552, 3545, 3538, 3531, 3524, 3516, 3509, 3502, 3495, 3488,
	 3480, 3473, 3466, 3459, 3452, 3445, 3437, 3430, 3423, 3416, 3408, 3401, 3394, 3387, 3380, 3372,
	 3365, 3358, 3351, 3344, 3336, 3329, 3322, 3315, 3307, 3300, 3293, 3286, 3278, 3271, 3264, 3257,
	 3249, 3242, 3235, 3228, 3220, 3213, 3206, 3199, 3191, 3184, 3177, 3169, 3162, 3155, 3148, 3140,
	 3133, 3126, 3118, 3111, 3104, 3097, 3089, 3082, 3075, 3067, 3060, 3053, 3045, 3038, 3031, 3024,
	 3016, 3009, 3002, 2994, 2987, 2980, 2972, 2965, 2958, 2950, 2943, 2936, 2928, 2921, 2914, 2906,
	 2899, 2892, 2884, 2877, 2870, 2862, 2855, 2848, 2840, 2833, 2826, 2818, 2811, 2803, 2796, 2789,
	 2781, 2774, 2767, 2759, 2752, 2744, 2737, 2730, 2722, 2715, 2708, 2700, 2693, 2685, 2678, 2671,
	 2663, 2656, 2648, 2641, 2634, 2626, 2619, 2611, 2604, 2597, 2589, 2582, 2574, 2567, 2560, 2552,
	 2545, 2537, 2530, 2522, 2515, 2508, 2500, 2493, 2485, 2478, 2470, 2463, 2456, 2448, 2441, 2433,
	 2426, 2418, 2411, 2404, 2396, 2389, 2381, 2374, 2366, 2359, 2351, 2344, 2336, 2329, 2322, 2314,
	 2307, 2299, 2292, 2284, 2277, 2269, 2262, 2254, 2247, 2239, 2232, 2224, 2217, 2209, 2202, 2195,
	 2187, 2180, 2172, 2165, 2157, 2150, 2142, 2135, 2127, 2120, 2112, 2105, 2097, 2090, 2082, 2075,
	 2067, 2060, 2052, 2045, 2037, 2030, 2022, 2015, 2007, 2000, 1992, 1984, 1977, 1969, 1962, 1954,
	 1947, 1939, 1932, 1924, 1917, 1909, 1902, 1894, 1887, 1879, 1872, 1864, 1857, 1849, 1841, 1834,
	 1826, 1819, 1811, 1804, 1796, 1789, 1781, 1774, 1766, 1758, 1751, 1743, 1736, 1728, 1721, 1713,
	 1706, 1698, 1690, 1683, 1675, 1668, 1660, 1653, 1645, 1637, 1630, 1622, 1615, 1607, 1600, 1592,
	 1584, 1577, 1569, 1562, 1554, 1547, 1539, 1531, 1524, 1516, 1509, 1501, 1494, 1486, 1478, 1471,
	 1463, 1456, 1448, 1440, 1433, 1425, 1418, 1410, 1402, 1395, 1387, 1380, 1372, 1364, 1357, 1349,
	 1342, 1334, 1326, 1319, 1311, 1304, 1296, 1288, 1281, 1273, 1266, 1258, 1250, 1243, 1235, 1228,
	 1220, 1212, 1205, 1197, 1189, 1182, 1174, 1167, 1159, 1151, 1144, 1136, 1129, 1121, 1113, 1106,
	 1098, 1090, 1083, 1075, 1068, 1060, 1052, 1045, 1037, 1029, 1022, 1014, 1007,  999,  991,  984,
	  976,  968,  961,  953,  945,  938,  930,  923,  915,  907,  900,  892,  884,  877,  869,  861,
	  854,  846,  838,  831,  823,  816,  808,  800,  793,  785,  777,  770,  762,  754,  747,  739,
	  731,  724,  716,  708,  701,  693,  686,  678,  670,  663,  655,  647,  640,  632,  624,  617,
	  609,  601,  594,  586,  578,  571,  563,  555,  548,  540,  532,  525,  517,  509,  502,  494,
	  486,  479,  471,  463,  456,  448,  440,  433,  425,  417,  410,  402,  394,  387,  379,  371,
	  364,  356,  348,  341,  333,  325,  318,  310,  302,  295,  287,  279,  272,  264,  256,  249,
	  241,  233,  226,  218,  210,  203,  195,  187,  180,  172,  164,  157,  149,  141,  134,  126,
	  118,  111,  103,   95,   88,   80,   72,   65,   57,   49,   42,   34,   26,   19,   11,    3
};


/* Register access permissions:
 * 0: none (reserved)
 * 1: read
 * 2: write
 * 3: read/write
 * 7: read^write (seperate functions/values)
 */
// Table that shows if a TMC475 register can also be read (1)
static const u8 TMC457RegisterReadable[128]=
{
//	0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0,    // 00 - 0F
	0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,    // 10 - 1F
	0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,    // 20 - 2F
	0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    // 30 - 3F
	1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    // 40 - 4F
	1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,    // 50 - 5F
	1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    // 60 - 6F
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1     // 70 - 7F
};

static s32 TMC457SoftwareCopy[128];  // Software copies of all TMC457 registers


/***************************************************************//**
	 \fn ReadWrite429(u8 *Read, u8 *Write)
	 \brief 40 bit SPI communication with TMC457
	 \param Read   five byte array holding the data read from the TMC457
	 \param Write  five byte array holding the data to write to the TMC457

	 This is the low-level function that does all SPI communication with
	 the TMC457. It sends a 40 bit SPI telegramme to the TMC457 and
	 receives the 40 bit answer telegramme from the TMC457.
********************************************************************/
static void ReadWrite457(u8 *Read, u8 *Write)
{
	Read[0] = ReadWriteSPI(SPI_DEV_TMC457, Write[0], FALSE);
	Read[1] = ReadWriteSPI(SPI_DEV_TMC457, Write[1], FALSE);
	Read[2] = ReadWriteSPI(SPI_DEV_TMC457, Write[2], FALSE);
	Read[3] = ReadWriteSPI(SPI_DEV_TMC457, Write[3], FALSE);
	Read[4] = ReadWriteSPI(SPI_DEV_TMC457, Write[4], TRUE);
}


/***************************************************************//**
	 \fn Write457Zero(u8 Address)
	 \brief Clear a TMC457 register
	 \param Address Address of the register

	  Write zero to a TMC457 register (and to its software copy).
********************************************************************/
void Write457Zero(u8 Address)
{
	u8 Write[5], Read[5];

	Write[0] = Address | TMC457_WRITE;
	Write[1] = 0;
	Write[2] = 0;
	Write[3] = 0;
	Write[4] = 0;

	ReadWrite457(Read, Write);
	TMC457SoftwareCopy[Address & 0x7F] = 0;
	if((Address & 0x7F) == TMC457_AMAX_DMAX)
	{
		TMC457SoftwareCopy[TMC457_AMAX] = 0;
		TMC457SoftwareCopy[TMC457_DMAX] = 0;
	}
}


/***************************************************************//**
	 \fn Write457Int(u8 Address, s32 Value)
	 \brief Write integer to a TMC457 register
	 \param Address   TMC457 register address
	 \param Value     Value to be written to the TMC457 register

	 Write a value to a TMC457 register (and to its software copy).
********************************************************************/
void Write457Int(u8 Address, s32 Value)
{
	u8 Write[5], Read[5];

	Write[0] = Address | TMC457_WRITE;
	Write[1] = Value >> 24;
	Write[2] = Value >> 16;
	Write[3] = Value >> 8;
	Write[4] = Value & 0xFF;

	ReadWrite457(Read, Write);
	TMC457SoftwareCopy[Address & 0x7F] = Value;
	if((Address & 0x7F) == TMC457_AMAX_DMAX)
	{
		TMC457SoftwareCopy[TMC457_AMAX] = Value;
		TMC457SoftwareCopy[TMC457_DMAX] = Value;
	}
}


/***************************************************************//**
	 \fn Write457Wavetable(u16 RAMAddress, u16 Value)
	 \brief Write to the TMC457 wavetable RAM
	 \param RAMAddress   Address into TMC457 wavetable RAM
	 \param Value        16 bit value to be written

	 Write a 16 bit value to the given location of the wavetable RAM
	 of the TMC457.
********************************************************************/
void Write457Wavetable(u16 RAMAddress, u16 Value)
{
	u8 Write[5], Read[5];

	Write[0] = TMC457_WAVETAB | TMC457_WRITE;
	Write[1] = RAMAddress >> 8;
	Write[2] = RAMAddress & 0xFF;
	Write[3] = Value >> 8;
	Write[4] = Value & 0xFF;

	ReadWrite457(Read, Write);
}


/***************************************************************//**
	 \fn Read457Int(u8 Address)
	 \brief Read TMC457 register
	 \param Address   register to be read from
	 \return contents of that register

	 Read a TMC457 register and return its value. If this is a
	 write-only register then the contents will be taken from the
	 software copy.
********************************************************************/
s32 Read457Int(u8 Address)
{
	u8 Write[5], Read[5];

	if(TMC457RegisterReadable[Address])
	{
		Write[0] = Address;
		Write[1] = 0;
		Write[2] = 0;
		Write[3] = 0;
		Write[4] = 0;

		ReadWrite457(Read, Write);

		return (Read[1]<<24) | (Read[2]<<16) | (Read[3]<<8) | (Read[4]);
	}
	else
	{
		return TMC457SoftwareCopy[Address];
	}
}


/***************************************************************//**
	 \fn Read457Wavetable(u16 RAMAddress)
	 \brief Read a value from the TMC457 wavetable RAM
	 \param RAMAddress   Address in wavetable RAM
	 \return Value read from the wavetable RAM

	 This function reads a 16 bit value from the TMC457 wavetable RAM.
********************************************************************/
u16 Read457Wavetable(u16 RAMAddress)
{
	u8 Write[5], Read[5];

	Write[0] = TMC457_WAVETAB;
	Write[1] = RAMAddress >> 8;
	Write[2] = RAMAddress & 0xFF;
	Write[3] = 0;
	Write[4] = 0;

	// Two SPI accesses are necessary for reading from the
	// wavetable RAM.
	ReadWrite457(Read, Write);
	ReadWrite457(Read, Write);

	return (Read[3]<<8) | Read[4];
}


/***************************************************************//**
	 \fn Set457RampMode(u32 RampMode)
	 \brief Set ramp mode of the TMC457
	 \param RampMode   Ramp mode to be set

	 Change the ramp mode of the TMC457. To be used with the
	 TMC457_RM_xxx constants.
********************************************************************/
void Set457RampMode(u32 RampMode)
{
	u32 ModeReg;

	ModeReg = Read457Int(TMC457_MODE) & 0xFFFFFFFC;
	ModeReg |= (RampMode & 0x00000003);
	Write457Int(TMC457_MODE, ModeReg);
}


/***************************************************************//**
	 \fn Init457Wavetable(u32 Resolution, s32 Offset)
	 \brief Initialize wavetable for the given microstep resolution
	 \param Resolution   Microstep resolution (0..11, 0=2048, 1=1024, ...)
	 \param Offset  Wavetable offset (mostly 0)

	 This function initializes the TMC457 wavetable for a given
	 microstep resolution. An offset can also be given (depending on
	 the motor this optimizes the zero crossing).
********************************************************************/
void Init457Wavetable(u32 Resolution, s32 Offset)
{
	u32 Address = 0;

	for(u32 i = 0; i < 8192; i += (1<<Resolution))
	{
		//Write457Wavetable(Address++, Offset+abs((s32) ((4095.0-Offset)*sin(6.28318530718*((double) i/8192.0)))));
		Write457Wavetable(Address++, Offset+((4095-Offset)*IntSinTable[MIRROR(i)])/10000);
	}
}


/***************************************************************//**
	 \fn Init457()
	 \brief Intialize the TMC457

	 This function does the necessary initializations on the TMC457.
********************************************************************/
void Init457(void)
{
	// Reset the TMC457 using the RESET pin (to be added by the user)

	// Initialize register software copies
	TMC457SoftwareCopy[TMC457_ENC_CONST] = 65536;
	TMC457SoftwareCopy[TMC457_PULSE_XSTEP_DIV] = 16;
	TMC457SoftwareCopy[TMC457_SEQ_DAC_SCALE] = 144;
	TMC457SoftwareCopy[TMC457_CHOP_CLK_DIV] = 640;

	// Set chopper frequency
	Write457Int(TMC457_CHOP_CLK_DIV, 0x01BB);

	// Set microstep resolution
	Write457Int(TMC457_SEQ_MODE, 0);

	// Load sine table
	Init457Wavetable(Read457Int(TMC457_SEQ_MODE) & 0x0F, 0);
}


/***************************************************************//**
	 \fn HardStop()
	 \brief Stop motor immediately.

	 This function stops the motor immediately, without using the
	 decelaration ramp.
********************************************************************/
void HardStop()
{
	u8 Flags;

	Flags = Read457Int(TMC457_MODE) & 0xFFFFFFFC;
	Write457Int(TMC457_MODE, TMC457_RM_HOLD | Flags);
	Write457Zero(TMC457_VTARGET);
}
