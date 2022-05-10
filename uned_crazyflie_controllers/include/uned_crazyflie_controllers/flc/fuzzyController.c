#ifdef __cplusplus
extern "C" {
#endif

#include "fuzzyController.h"
#include <stdio.h>

float fuzzyController_evaluateMF(pFuzzyController_MF_t pMF, float value)
{
    float weight = 0;

    switch(pMF->type)
    {
        case(MF_CONST):
            weight = pMF->p1;
            break;

        case(MF_GAMMA):
            if (value <= pMF->p1)
            {
                weight = 0;
            }
            else if (value >= pMF->p2)
            {
                weight = 1;
            }
            else
            {
                weight = (value - pMF->p1) / (pMF->p2 - pMF->p1);
            }
            break;

        case(MF_EL):
            if (value <= pMF->p1)
            {
                weight = 1;
            }
            else if (value >= pMF->p2)
            {
                weight = 0;
            }
            else
            {
                weight = (value - pMF->p2) / (pMF->p1 - pMF->p2);
            }
            break;

        case(MF_LAMBDA):
            if (value <= pMF->p1)
            {
                weight = 0;
            }
            else if (value >= pMF->p3)
            {
                weight = 0;
            }
            else if (value <= pMF->p2)
            {
                weight = (value - pMF->p1) / (pMF->p2 - pMF->p1);
            }
            else
            {
                weight = (value - pMF->p3) / (pMF->p2 - pMF->p3);
            }
            break;

        case(MF_PI):
            if (value <= pMF->p1)
            {
                weight = 0;
            }
            else if (value >= pMF->p4)
            {
                weight = 0;
            }
            else if (value <= pMF->p2)
            {
                weight = (value - pMF->p1) / (pMF->p2 - pMF->p1);
            }
            else if (value >= pMF->p3)
            {
                weight = (value - pMF->p4) / (pMF->p3 - pMF->p4);
            }
            else
            {
                weight = 1;
            }
            break;

        default:
            break;
    }

    return weight;
}
void fuzzyController_evaluatePertenence(pFuzzyController_t Handler, float *inputs)
{
	pFuzzyController_input_t pInput = Handler->pInput;

    for(int i = 0; i < Handler->inputsNumber; i++)
    {
        int mfNum = pInput->mfNumber;
        for(int j = 0; j < mfNum; j++)
        {
        	pInput->pPertenence[j] = fuzzyController_evaluateMF(&(pInput->pMF[j]), inputs[i]);
        }
        pInput++;
    }
}

void fuzzyController_evaluateRules(pFuzzyController_t Handler)
{
	printf("\nNumero de MF de salida: %d", Handler->pOutput->mfNumber);
	for(int i = 0; i < Handler->pOutput->mfNumber; i++)
	{
		printf("\nPoner a 0 weight %d: %f", i, Handler->pOutput->pWeight[i]);
		Handler->pOutput->pWeight[i] = 0;
		printf("\nNuevo valor weight %d: %f", i, Handler->pOutput->pWeight[i]);
	}

    for(int i = 0; i < Handler->rulesNumber; i++)
    {
        float temp;
        uint8_t in1 = Handler->pRule[i].input1;
        uint8_t mf1 = Handler->pRule[i].mf1;
        uint8_t in2 = Handler->pRule[i].input2;
        uint8_t mf2 = Handler->pRule[i].mf2;
        uint8_t mfOut = Handler->pRule[i].mfOut;
        printf("\nRegla %d: MF1: %f, MF2: %f", i, Handler->pInput[in1].pPertenence[mf1], Handler->pInput[in2].pPertenence[mf2]);
        if ((0 != Handler->pInput[in1].pPertenence[mf1]) &&
            (0 != Handler->pInput[in2].pPertenence[mf2]))
        {
        	printf("\nRegla %d: Valores de pertenencia mayores de 0", i);
            if ((Handler->pInput[in1].pPertenence[mf1]) >=
             (Handler->pInput[in2].pPertenence[mf2]))
            {
                temp = Handler->pInput[in2].pPertenence[mf2];
            }
            else
            {
                temp = Handler->pInput[in1].pPertenence[mf1];
            }
            if (temp > Handler->pOutput->pWeight[mfOut])
            {
                Handler->pOutput->pWeight[mfOut] = temp;
            }
        }
    }
}

float fuzzyController_calculateOutput(pFuzzyController_t Handler)
{
    float num = 0;
    float den = 0;

    for(int i = 0; i < Handler->pOutput->mfNumber; i++)
    {
        num += (Handler->pOutput->pWeight[i] * Handler->pOutput->pMF[i].p1);
        den += Handler->pOutput->pWeight[i];
    }

    return (num/den);
}

float fuzzyController_iterate(pFuzzyController_t Handler, float *inputs)
{
    fuzzyController_evaluatePertenence(Handler, inputs);
    fuzzyController_evaluateRules(Handler);

    return fuzzyController_calculateOutput(Handler);
}

#ifdef __cplusplus
}
#endif
