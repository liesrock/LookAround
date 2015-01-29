#include <headControllerTest.hpp>


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


robotPart::robotPart( )
{

}

void robotPart::configurePart(const char *portName)
{
    out_port.open(portName);
    cout << "portname: " << portName << endl;
    robotPartDriver.view(iEncs);
    robotPartDriver.view(ctrlMode);
    robotPartDriver.view(posControl);
    robotPartDriver.view(posDir);
    robotPartDriver.view(velControl);
    robotPartDriver.view(limits);
}

bool robotPart::configureJoint(int j, int _type, double _amplitude, double _time, double _period)
{
    // per più giunti control diventa un vettore di puntatori.
    control = new VelCtrl(j);
    double max, min;
    limits->getLimits(control->joint, &min, &max);

    control->home = min + ( (max - min)/2);
    cout << "limits are : min " << min << " max " << max << " home " << control->home << std::endl;

    homing();

    control->configure( _type, _amplitude, _time, _period, control->home);
}

void robotPart::update()
{
    // per più giunti fare un ciclo for

//     double _velocity = control->compute();
//     printf("velocity = %f\n", _velocity);
//     velControl->velocityMove(control->joint, _velocity);
//     Bottle msg;
//     msg.addDouble(_velocity);
//     out_port.write(msg);
    
    double posRef = control->compute();
    printf("new ref = %f\n", posRef);
    posDir->setPosition(control->joint, posRef);

    Bottle msg;
    msg.addDouble(posRef);
    out_port.write(msg);
}


void robotPart::homing()
{
    // se più giunti usare ciclo for

//     posControl->positionMove(control->joint, control->home);
    
    ctrlMode->setControlMode(control->joint, VOCAB_CM_POSITION_DIRECT);

    bool done =  false;
    int trials = 0;
    double enc = 0;
    int sign = 0;
    double step_size = 0.2;
    
    iEncs->getEncoder(control->joint, &enc);

    if(control->home > enc)
	sign = +1;
    else
	sign = -1;
    
    cout << "homing joint " << control->joint << " to pos " << control->home << "; current pos " << enc << " sign " << sign << endl;

    double new_ref = enc;

    while(!done)
    {
	iEncs->getEncoder(control->joint, &enc);
	new_ref += step_size * sign;
	
	posDir->setPosition(control->joint, new_ref);

        cout << "homing joint " << control->joint << " to pos " << control->home << "; current pos " << "new_ref  "  << new_ref << endl;
        //posControl->checkMotionDone(control->joint, &done);
	if( (fabs(enc - control->home)) <= 0.5)
	{
	    std::cout << "suca" << std::endl; 
	    done = true;
	}	
        Time::delay(0.1);
    }
}


// This class controls a single joint using the velocity controller.
// Ideally could be instatiated as many times as needed to control every joint of the robotpart
// indipendently and with different parameter... let see if this can be done easily

VelCtrl::VelCtrl(int _joint)
{
    joint = _joint;
    index = 0;
    amp = 0;
    period = 0;
    time = 0;
    type = 0;
};



bool VelCtrl::configure(int _type, double _amplitude, double _time, double _period, double _home)
{
    type = _type;
    amp = _amplitude;
    time = _time;
    period = _period;
    home = _home;


    // usare dei puntatori a funzione per impostare la computeXXX e togliere lo switch dalla compute()
    switch(type)
    {
        // default case is ON TOP and BREAK IS MISSING on purpose, so it'll fallback onto 0->sinusoidal
        default:
            printf("Error! unknown waveform type %d. Using sinusoidal!\n", type);

        // sinusoidale
        case 0:

            break;

        // trapezoidale
        case 1:

            break;

        // onda quadra
        case 2:

            break;
    }
    return true;
};

double VelCtrl::compute()
{
    // usare puntatore a funzione o simili
    index++;
    return sinusoidal(index);
}

double VelCtrl::sinusoidal(int index)
{
    //cout << "amp " << amp << " index " << index << " time " << time << " period " << period;
    return amp*sin(2*(PI) * index *time/period);
}


LookModule::LookModule()
{
    cout << "LookModule constructor" << endl;
    updatePeriod = 0.02;
};

LookModule::~LookModule()
{
    cout << "LookModule destructor" << endl;
};

double LookModule::getPeriod()
{
    return updatePeriod;
};

bool LookModule::updateModule()
{
    static int up=0;
    if(up==0)
    {
        up++;
        cout << "LookModule updateModule...called every period" << endl;
    }
    part.update();
    return true;
};

bool LookModule::open(Searchable& config)
{
    cout << "LookModule open" << endl;
    return true;
};

bool LookModule::close()
{
    cout << "LookModule close" << endl;
    return true;
};

bool LookModule::configure(yarp::os::ResourceFinder &rf)
{
    cout << "LookModule configure" << endl;


///////////////// reading config file //////////////////////////////////

    // port names
    ConstString prefix = "/";
    ConstString robotName = prefix + rf.check("robot",Value("robot"),"Getting robot name").asString().c_str();
    ConstString partName  = rf.find("part").asString().c_str();
    ConstString portName = (robotName + "/" + partName);
    ConstString outPortName = (prefix + "velInterface/" + partName);

    // vel control params
    int joint =  rf.check("joint", Value(0), "joint to be controlled").asInt();
    int type  =  rf.check("type", Value(0), "type of velocity profile").asInt();   // 0= sinusoidale, 1=gradino...
    int amplitude =  rf.check("amp", Value(10), "max velocity in degrees/sec").asInt();
    int period    =  rf.check("period", Value(1), "period of the waveform in sec").asInt();


    //  printa la configurazione per vedere se è corretta
    cout << "Running with:" << endl;
    cout << "robot: " << robotName.c_str() << endl;
    cout << "part:  " << partName.c_str()  << endl;
    cout << "port:  " << portName.c_str()  << endl;
    cout << "joint: " << joint << endl;
    cout << "type:  " << type << endl;
    cout << "ampli: " << amplitude << endl;
    cout << "period:" << period << endl;


    if(robotName == "" )
    {
        printf("Error!! specify a robot name\n");
        return -1;
    }

    if(partName == "" )
    {
        printf("Error!! specify a part name\n");
        return -1;
    }


 ///////////// connettiti col robot
    Property options;
    options.put("device", "remote_controlboard"); // aggiungo parametri necessari
    options.put("local", "...");
    options.put("remote", portName);


    part.robotPartDriver.open(options);
    if(!part.robotPartDriver.isValid())
    {
        printf("Cannot connect to the robot part\n");
        return false;
    }
    else
        printf("remote control board opened succesfully\n");

    cout << "before configure port" <<  outPortName.c_str() << endl;
    part.configurePart(outPortName.c_str());
    part.configureJoint(joint, type, amplitude, updatePeriod, period);
    return true;
};


bool LookModule::interruptModule()
{
    cout << "quitting";
    part.out_port.close();
    part.homing();
    return true;
}

int main(int argc, char **argv)
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"YARP server not available!\n");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("walkman/conf/vel_control");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

////////////////////////////////////////////////////////
    // configure module
    LookModule module;

    // instanzia il mio oggetto e vai!!
    return module.runModule(rf);
}
