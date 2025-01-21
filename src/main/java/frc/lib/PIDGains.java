package frc.lib;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class PIDGains {
  public final double p;
  public final double i;
  public final double d;

  public PIDGains(double _p, double _i, double _d) {
    p = _p;
    i = _i;
    d = _d;
  }

  // public static void setSparkMaxGains(SparkPIDController  _controller, PIDGains _gains) {
  //   _controller.setP(_gains.p);
  //   _controller.setI(_gains.i);
  //   _controller.setD(_gains.d);
  // }

    public static void setSparkMaxGains(SparkClosedLoopController _controller, PIDGains _gains) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder) //TODO: REVISIT THIS!
      .pid(_gains.p, _gains.i, _gains.d);
  }
}
