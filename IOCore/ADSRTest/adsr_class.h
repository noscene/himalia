#ifndef ADSR_CLASS_H
#define ADSR_CLASS_H

// http://www.earlevel.com/main/2013/06/01/envelope-generators/
// https://github.com/fdeste/ADSR/blob/master/ADSR.c

class ZM_ADSR {
    public:

    enum envState {
        env_idle = 0, env_attack, env_decay,  env_sustain, env_release
    };

    envState    state;
	double      output;
	double      attackRate;
	double      decayRate;
	double      releaseRate;
	double      attackCoef;
	double      decayCoef;
	double      releaseCoef;
	double      sustainLevel;
    double      targetRatioA;
    double      targetRatioDR;
    double      attackBase;
    double      decayBase;
    double      releaseBase;
	int         gate;

    float  lut_log[4096];
    float  lut_exp[4096];
    int     end_of_attack = 0;
    int     end_of_decay  = 0;

    ZM_ADSR() {


        for(int i = 0 ; i < 4096 ; i++ ){
            lut_log[i] = logf( (float)i / 512.0 );   // 0.0 ..... 8.0
            lut_exp[i] = expf( (float)i / 512.0 - 4.0 );  // -4.0 .... 4.0
        }

        setAttackRate(0.3);
        setDecayRate(0.2);
        setReleaseRate(0.5);
        setSustainLevel(0.5);
        setTargetRatioA(0.3);
        setTargetRatioDR(0.01);
    };


    float calcCoefL(float rate, float targetRatio) {  
        if(rate <= 0.0f)
            return 0.0f;

        uint16_t    x1 = (1.0 + targetRatio) / targetRatio * 512.0;
        float       x2 = lut_log[x1 & 0x0fff] / rate + 4.0;
        uint16_t    x3 = -(x2 * 512.0) ;
        return      lut_exp[x3 & 0x0fff];
        // return expf(-logf( )/ rate); // need a lot of time about 70uS
    };


    float calcCoef(float rate, float targetRatio) {  
        if(rate <= 0.0f)
            return 0.0f;
        return 1.0/(powf(1.0 / targetRatio + 1.0 , 1.0 / rate)); // may Faster ???
        // return expf(-logf((1.0 + targetRatio) / targetRatio) / rate); // need a lot of time about 70uS
    };

    void setAttackRate( double rate) {
        attackRate = rate;
        attackCoef = calcCoef(rate, targetRatioA);
        attackBase = (1.0 + targetRatioA) * (1.0 - attackCoef);
    };

    void setDecayRate(double rate) {
        decayRate = rate ;
        decayCoef = calcCoef(rate, targetRatioDR);
        decayBase = (sustainLevel - targetRatioDR) * (1.0 - decayCoef);
    };

    void setReleaseRate( double rate) {
        releaseRate = rate ;
        releaseCoef = calcCoef(rate, targetRatioDR);
        releaseBase = -targetRatioDR * (1.0 - releaseCoef);
    };

    void setSustainLevel( double level) {
        sustainLevel = level;
        decayBase = (sustainLevel - targetRatioDR) * (1.0 - decayCoef);
    };

    void setTargetRatioAll( double targetRatio) {
        setTargetRatioA( targetRatio );
        setTargetRatioDR(  targetRatio );
    };
    
    void setTargetRatioA( double targetRatio) {
        if (targetRatio < 0.000000001)
            targetRatio = 0.000000001;  // -180 dB
        targetRatioA = targetRatio;
        attackBase = (1.0 + targetRatioA) * (1.0 - attackCoef);
    };

    void setTargetRatioDR( double targetRatio) {
        if (targetRatio < 0.000000001)
            targetRatio = 0.000000001;  // -180 dB
        targetRatioDR = targetRatio;
        decayBase = (sustainLevel - targetRatioDR) * (1.0 - decayCoef);
        releaseBase = -targetRatioDR * (1.0 - releaseCoef);
    };

    double process() {

        if(end_of_attack > 0){
            end_of_attack--;
            PORT->Group[PORTA].OUTSET.reg = 1ul << 17; 
        }else{
            PORT->Group[PORTA].OUTCLR.reg = 1ul << 17; 
        }


        if(end_of_decay > 0){
            end_of_decay--;
            PORT->Group[PORTA].OUTSET.reg = 1ul << 16; 
        }else{
            PORT->Group[PORTA].OUTCLR.reg = 1ul << 16; 
        }

        switch (state) {
            case env_idle:
                break;
            case env_attack:
                output = attackBase + output * attackCoef;
                if (output >= 1.0) {
                    output = 1.0;
                    state = env_decay;
                    end_of_attack=60;
                }
                break;
            case env_decay:
                output = decayBase + output * decayCoef;
                if (output <= sustainLevel) {
                    output = sustainLevel;
                    state = env_sustain;
                    end_of_decay=60;
                }
                break;
            case env_sustain:
                output = sustainLevel;
                break;
            case env_release:
                output = releaseBase + output * releaseCoef;
                if (output <= 0.0) {
                    output = 0.0;
                    state = env_idle;
                }
        }
        return output;
    };

    // Note On / Off
    void setNewGateState(bool newGate) {
        if (newGate ) {
            state = env_attack;
            gate = 1;
        }
        else if (state != env_idle){
            state = env_release;
            gate = 0;
        }
    };

    int getState() {
        return state;
    };

    void reset() {
        gate = 0;
        state = env_idle;
        output = 0.0;
    };

    double getOutput() {
        return output;
    };

};



#endif