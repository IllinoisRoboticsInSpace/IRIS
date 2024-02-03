#ifndef _OPERATOR_
#define _OPERATOR_

class MotorOperator
{
    public:
        // constructor
        MotorOperator(bool inverted, bool enabled);
        MotorOperator();
        MotorOperator(const MotorOperator& other);
        MotorOperator& operator=(const MotorOperator& other);

        // general
        void setInverted(bool inverted);
        void setEnabled(bool enabled);
        bool getInverted();
        bool getEnabled();

        // (pure) virtual
        virtual bool init() = 0;

    protected:
        // general
        bool inverted;
        bool enabled;

};

#endif
