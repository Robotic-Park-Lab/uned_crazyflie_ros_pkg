#ifndef __FLC_H
#define __FLC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//Membership Functions types
typedef enum fuzzyController_MF_type
{
    MF_CONST = 0,
    MF_GAMMA,
    MF_EL,
    MF_LAMBDA,
    MF_PI
}fuzzyController_MF_type_t;

typedef struct fuzzyController_MF
{
    fuzzyController_MF_type_t type;
    float p1;
    float p2;
    float p3;
    float p4;
}fuzzyController_MF_t, *pFuzzyController_MF_t;

typedef struct fuzzyController_input
{
    int mfNumber;
    pFuzzyController_MF_t pMF;
    float *pPertenence;
}fuzzyController_input_t, *pFuzzyController_input_t;

typedef struct fuzzyController_output
{
    int mfNumber;
    pFuzzyController_MF_t pMF;
    float *pWeight;
}fuzzyController_output_t, *pFuzzyController_output_t;

typedef struct fuzzyController_rule
{
    uint8_t input1;
    uint8_t mf1;
    uint8_t input2;
    uint8_t mf2;
    uint8_t mfOut;
}fuzzyController_rule_t, *pFuzzyController_rule_t;

typedef struct fuzzyController
{
    int inputsNumber;
    pFuzzyController_input_t pInput;
    pFuzzyController_output_t pOutput;
    int rulesNumber;
    pFuzzyController_rule_t pRule;
}fuzzyController_t, *pFuzzyController_t;

/******************************************************************************
 * 
 * Common MACROS
 * 
 *****************************************************************************/

/*
 #define FUZZY_CONTROLLER_HADLER_INIT(fcHandlerName, inputNum, rulesNum) \
// 	fuzzyController_input_t fc_Inputs_##fcHandlerName[inputNum]; \
// 	fuzzyController_output_t fc_Output_##fcHandlerName; \
// 	fuzzyController_rule_t fc_Rules_##fcHandlerName[rulesNum]; \
// 	fuzzyController_t fcHandlerName = { \
// 			.inputsNumber = inputNum, \
// 			.pInput = fc_Inputs_##fcHandlerName, \
// 			.pOutput = &fc_Output_##fcHandlerName, \
// 			.rulesNumber = rulesNum, \
// 			.pRule = fc_Rules_##fcHandlerName}
*/

#define FUZZY_CONTROLLER_HADLER_INIT(fcHandlerName, inputNum, rulesNum) \
	fuzzyController_input_t fc_Inputs_##fcHandlerName[inputNum]; \
	fuzzyController_output_t fc_Output_##fcHandlerName; \
	fuzzyController_rule_t fc_Rules_##fcHandlerName[rulesNum]; \
	fuzzyController_t fcHandlerName = { \
			inputNum, \
			fc_Inputs_##fcHandlerName, \
			&fc_Output_##fcHandlerName, \
			rulesNum, \
			fc_Rules_##fcHandlerName}

#define FUZZY_CONTROLLER_ADD_MF(varName, mfIndex, mfType, pt1, pt2, \
		pt3, pt4) \
		fc_MF_##varName[mfIndex].type = mfType; \
		fc_MF_##varName[mfIndex].p1 = pt1; \
		fc_MF_##varName[mfIndex].p2 = pt2; \
		fc_MF_##varName[mfIndex].p3 = pt3; \
		fc_MF_##varName[mfIndex].p4 = pt4

#define FUZZY_CONTROLLER_ADD_RULE(fcHandlerName, ruleIndex, IN1, MF1, \
		IN2, MF2, MFOUT) \
		fc_Rules_##fcHandlerName[ruleIndex].input1 = IN1; \
		fc_Rules_##fcHandlerName[ruleIndex].mf1 = MF1; \
		fc_Rules_##fcHandlerName[ruleIndex].input2 = IN2; \
		fc_Rules_##fcHandlerName[ruleIndex].mf2 = MF2; \
		fc_Rules_##fcHandlerName[ruleIndex].mfOut = MFOUT

/******************************************************************************
 * 
 * MACROS For defining FLC outside of a function
 * 
 *****************************************************************************/

#define FUZZY_CONTROLLER_ADD_INPUT(fcHandlerName, inIndex, inName, mfNum) \
		fuzzyController_MF_t fc_MF_##inName[mfNum]; \
		float fc_Pertenence_##inName[mfNum]; \
		fc_Inputs_##fcHandlerName[inIndex].mfNumber = mfNum; \
		fc_Inputs_##fcHandlerName[inIndex].pMF = fc_MF_##inName; \
		fc_Inputs_##fcHandlerName[inIndex].pPertenence = fc_Pertenence_##inName

#define FUZZY_CONTROLLER_ADD_OUTPUT(fcHandlerName, outName, mfNum) \
	fuzzyController_MF_t fc_MF_##outName[mfNum]; \
	float weights_##outName[mfNum]; \
	fc_Output_##fcHandlerName.mfNumber = mfNum; \
	fc_Output_##fcHandlerName.pMF = fc_MF_##outName; \
	fc_Output_##fcHandlerName.pWeight = weights_##outName

/******************************************************************************
 * 
 * MACROS For defining FLC inside a function
 * 		--> Definition must be done outside a function or methode
 * 		--> Initialization can be done into or outside a funciton or methode
 * 
 *****************************************************************************/

#define FUZZY_CONTROLLER_DEFINE_INPUT(fcHandlerName, inName, mfNum) \
		fuzzyController_MF_t fc_MF_##inName[mfNum]; \
		float fc_Pertenence_##inName[mfNum]

#define FUZZY_CONTROLLER_INIT_INPUT(fcHandlerName, inName, mfNum, inIndex) \
		fc_Inputs_##fcHandlerName[inIndex].mfNumber = mfNum; \
		fc_Inputs_##fcHandlerName[inIndex].pMF = fc_MF_##inName; \
		fc_Inputs_##fcHandlerName[inIndex].pPertenence = fc_Pertenence_##inName


#define FUZZY_CONTROLLER_DEFINE_OUTPUT(fcHandlerName, outName, mfNum) \
	fuzzyController_MF_t fc_MF_##outName[mfNum]; \
	float weights_##outName[mfNum]

#define FUZZY_CONTROLLER_INIT_OUTPUT(fcHandlerName, outName, mfNum) \
	fc_Output_##fcHandlerName.mfNumber = mfNum; \
	fc_Output_##fcHandlerName.pMF = fc_MF_##outName; \
	fc_Output_##fcHandlerName.pWeight = weights_##outName

/******************************************************************************
 * 
 * Public functions
 * 
 *****************************************************************************/

float fuzzyController_iterate(pFuzzyController_t Handler, float *inputs);

#ifdef __cplusplus
}
#endif

#endif /* __FLC_H */