#include <robot/bit.h>

namespace kinematics
{

union SFLG1
{
    unsigned int val;
    struct STRFLG
    {
        unsigned int NF : 1;
        unsigned int AB : 1;
        unsigned int RL : 1;
    }bit;

    SFLG1(){ val=0; }

    SFLG1 operator=(const SFLG1& obj)
    {
        this->val = obj.val;
        return(*this);
    }

    void SET(int _n, bool _val)
    {
        switch(_n)
        {
            case 0: this->bit.NF = _val; break;
            case 1: this->bit.AB = _val; break;
            case 2: this->bit.RL = _val; break;
            default:
            {
                std::cerr << "[SFLG1] wrong index" << std::endl;
                assert(false);
            }
        }
    }
};

union SFLG2
{
    unsigned int val;
    struct ROTFLG
    {
        unsigned int j1 : 4;
        unsigned int j2 : 4;
        unsigned int j3 : 4;
        unsigned int j4 : 4;
        unsigned int j5 : 4;
        unsigned int j6 : 4;
        unsigned int j7 : 4;
        unsigned int j8 : 4;
    }jnt;
    
    SFLG2(){val=0;}

    SFLG2 operator=(const SFLG2& obj)
    {
        this->val = obj.val;
        return(*this);
    }

    void SET(int _n, int _val)
    {
        assert(-8<_val && _val<8);
        switch(_n)
        {
            case 0: this->jnt.j1 = (_val<0) ? (_val+16) : (_val); break;
            case 1: this->jnt.j2 = (_val<0) ? (_val+16) : (_val); break;
            case 2: this->jnt.j3 = (_val<0) ? (_val+16) : (_val); break;
            case 3: this->jnt.j4 = (_val<0) ? (_val+16) : (_val); break;
            case 4: this->jnt.j5 = (_val<0) ? (_val+16) : (_val); break;
            case 5: this->jnt.j6 = (_val<0) ? (_val+16) : (_val); break;
            case 6: this->jnt.j7 = (_val<0) ? (_val+16) : (_val); break;
            case 7: this->jnt.j8 = (_val<0) ? (_val+16) : (_val); break;
            default:
            {
                std::cerr << "[SFLG2] wrong index" << std::endl;
                assert(false);
            }
        }
    }
};

struct SFLG
{
    SFLG1 flg1;
    SFLG2 flg2;
};

inline std::ostream& operator<<(std::ostream& stream, SFLG1& obj)
{
    char cData[512];
    sprintf(cData,"(NF, AB, RL) = (%d, %d, %d)", obj.bit.NF, obj.bit.AB, obj.bit.RL);
    return( stream << cData);
}

inline std::ostream& operator<<(std::ostream& stream, SFLG2& obj)
{
    char cData[512];
    sprintf(cData,"SFLG2 = (%d, %d, %d, %d, %d, %d, %d, %d)"
                 , obj.jnt.j1, obj.jnt.j2, obj.jnt.j3, obj.jnt.j4
                 , obj.jnt.j5, obj.jnt.j6, obj.jnt.j7, obj.jnt.j8);
    return( stream << cData);
}

}