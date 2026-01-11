package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

public class MutSlewRateLimiter {
   private double m_positiveRateLimit;
   private double m_negativeRateLimit;
   private double m_positiveDecelLimit;
   private double m_negativeDecelLimit;
   private double m_prevVal;
   private double m_prevTime;

   public MutSlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
      this.m_positiveRateLimit = positiveRateLimit;
      this.m_negativeRateLimit = negativeRateLimit;
      this.m_positiveDecelLimit = positiveRateLimit;
      this.m_negativeDecelLimit = negativeRateLimit;
      this.m_prevVal = initialValue;
      this.m_prevTime = MathSharedStore.getTimestamp();
   }

   public MutSlewRateLimiter(double rateLimit) {
      this(rateLimit, -rateLimit, 0.0);
   }

   public double calculate(double input) {
      double currentTime = MathSharedStore.getTimestamp();
      double elapsedTime = currentTime - this.m_prevTime;

      double wantedChange = MathUtil.clamp(input - this.m_prevVal, this.m_negativeRateLimit * elapsedTime, this.m_positiveRateLimit * elapsedTime);
      if (Math.abs(this.m_prevVal - wantedChange) > Math.abs(this.m_prevVal)){
        double wantedDeccel = MathUtil.clamp(input - this.m_prevVal, this.m_negativeDecelLimit * elapsedTime, this.m_positiveDecelLimit * elapsedTime);
        this.m_prevVal += wantedDeccel;
        this.m_prevTime = currentTime;
        return this.m_prevVal;
      }
      this.m_prevVal += wantedChange;
      this.m_prevTime = currentTime;
      return this.m_prevVal;
   }

   public double lastValue() {
      return this.m_prevVal;
   }

   /**
    * expects positive + and negative - numbers, not just magnitudes
    * @param positiveRateLimit
    * @param negativeRateLimit
    */
   public void setRateLimit(double positiveRateLimit, double negativeRateLimit){
     this.m_positiveRateLimit = positiveRateLimit;
     this.m_negativeRateLimit = negativeRateLimit;
   }

   public void setRateLimit(double rateLimit){
     setRateLimit(rateLimit, -rateLimit);
   }

   public void setDecelLimit(double positiveDecelLimit, double negativeDecelLimit){
     this.m_positiveDecelLimit = positiveDecelLimit;
     this.m_negativeDecelLimit = negativeDecelLimit;
   }

   public void setDecelLimit(double decelLimit){
     setDecelLimit(decelLimit, -decelLimit);
   }


   public void reset(double value) {
      this.m_prevVal = value;
      this.m_prevTime = MathSharedStore.getTimestamp();
   }
}
