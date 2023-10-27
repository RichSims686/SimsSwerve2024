package frc.robot.util.led.animation;

import frc.robot.util.led.functions.Gradient;
import frc.robot.util.led.functions.TilingFunction;
import frc.robot.util.led.strips.LEDStrip;

public class FlashingAnimation extends LEDAnimation {
    private final LEDStrip[] strips;
    private final Gradient gradient;
    private final TilingFunction tilingFunction;

    private double              period = 1;
    public double               getPeriod()                 {return period;}
    public FlashingAnimation    setPeriod(double period)    {this.period = period; return this;}

    public FlashingAnimation(Gradient gradient, LEDStrip... strips) {
        this(gradient, TilingFunction.Sawtooth, strips);
    }
    public FlashingAnimation(Gradient gradient, TilingFunction tilingFunction, LEDStrip... strips) {
        this.strips = strips;
        this.gradient = gradient;
        this.tilingFunction = tilingFunction;
    }

    @Override
    protected void runAnimation(LEDManager manager) {
        for(LEDStrip ledStrip : strips) {
            ledStrip.foreach((int i) -> {
                ledStrip.setLED(i, gradient.getColor(tilingFunction.tile(animationTimer.get()/period)));
            });
        }
    }
}
