namespace Control
{
    class PID 
    {
    public:
        double kp;
        double ki;
        double kd;
        double error_out;
        double last_error;
        double integral;
        double error_temp = 0;
        double inte_max;//积分限副
        double last_diff;//上一次微分值

        double PIDPositional(double error);//位置式PID控制器
        double PIDIncremental(double error);
        void Init();
        
    };
    
    double PID::PIDPositional(double error)//位置式PID控制器
    {
        integral += error;
 	error_temp = error;
        error = abs(M_PI / 2-error);
        if(integral>inte_max)
            integral = inte_max;

        error_out = (kp*error) + (ki*integral) + (kd*(error-last_error));
        if(error_temp > M_PI / 2)
	error = 90 + error_out;
	//ROS_INFO("error_out:%f",error_out);
	if(error_temp < M_PI / 2)
	error = 90 - error_out;
        last_error = error;
        return error;
    }
    
    double PID::PIDIncremental(double error)//增量式PID控制器
    {
       
        error_out = kp*(error-last_error) + ki*error + kd*((error-last_error)-last_diff);

        last_diff = error - last_error;
        last_error = error;

        return error;
    }

    void PID::Init()//PID初始化,参数在此调节
    {
    // 34.5 0.25 31.5 0.15eyd
        kp = 55; //6 5 20.0 弯道效果较好  25.0所有最好 30-32.5 31.5    (32.5跑第二圈的导航)   85(1740)  115(1776)    105(1760 1770 1780)
        ki = 0.0; // 0.0 弯道效果比较好     0.0005（1776）
        kd = 0.003;//3 5  5.0 弯道效果比较好  0.1所有最好 0.1-0.15 0.15      0.15   0.0155(1740)  0.0135(1776)   0.0135（1776）     0.02(1760  1770 1780)   0.03
        error_out = 0.0;
        last_error = 0.0;
        integral = 0.0;
        inte_max = 8.0;
        last_diff = 0.0;
        
        // kp =18.0;
        // ki = 0.1;
        // kd = 3.0;
        // error_out = 0.0;
        // last_error = 0.0;
        // integral = 0.0;
        // inte_max = 8.0;
        // last_diff = 0.0;
    }
    
}
