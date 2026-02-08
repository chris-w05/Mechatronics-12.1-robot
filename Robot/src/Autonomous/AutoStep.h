// AutoStep.h

//This is a template that all autonomous steps must follow for standardization. 
//All autonomous steps are also AutoSteps. 
#ifndef AUTO_STEP_H
#define AUTO_STEP_H


//Generic step for assembling autonomous routines
class AutoStep {
public:
    virtual ~AutoStep() = default;

    virtual void start() = 0;
    virtual void update() = 0;
    virtual bool isFinished() const = 0;
    virtual void end() = 0;
};

#endif
