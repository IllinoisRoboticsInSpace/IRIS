#ifndef _OPERATOR_
#define _OPERATOR_

class MotorOperator
{
    public:
        // general
        void setInverted(bool inverted);
        void setEnabled(bool enabled);
        bool getEnabled();

        // virtual
        virtual void init();

    private:
        // general
        bool inverted;
        bool enabled;

};

#endif
