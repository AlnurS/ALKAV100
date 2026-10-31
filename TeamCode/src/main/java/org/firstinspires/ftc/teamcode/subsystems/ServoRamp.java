package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Подсистема для управления сервоприводом рампы (ServoRamp).
 * Оптимизирована для goBILDA 300° Dual Mode Servo (PWM диапазон 500us - 2500us).
 * Автоматически ищет сервопривод "SERVO RAMPA" и его вариации в конфигурации Driver Hub.
 */
public class ServoRamp {
    private Servo servo;
    private CRServo crServo;
    private String attachedName = "NOT CONNECTED";

    public void init(HardwareMap hardwareMap) {
        servo = null;
        crServo = null;
        attachedName = "NOT CONNECTED";

        // Список возможных имён сервопривода в Robot Configuration на Driver Hub
        String[] possibleNames = {
            RobotConstants.RAMP_SERVO_NAME,
            "SERVO RAMPA",
            "SERVO_RAMPA",
            "servoRampa",
            "servo rampa",
            "servoRamp",
            "rampServo",
            "ramp"
        };

        // 1. Поиск как обычного позиционного сервопривода (Servo)
        for (String name : possibleNames) {
            try {
                servo = hardwareMap.get(Servo.class, name);
                if (servo != null) {
                    attachedName = name;
                    break;
                }
            } catch (Exception ignored) {
            }
        }

        // 2. Настройка PWM диапазона для goBILDA Dual Mode Servo (500us - 2500us -> полный угол 300°)
        if (servo instanceof ServoImplEx) {
            try {
                ServoImplEx servoEx = (ServoImplEx) servo;
                servoEx.setPwmEnable();
                servoEx.setPwmRange(new PwmControl.PwmRange(500, 2500));
            } catch (Exception ignored) {
            }
        }

        // 3. Если не найден как Servo, ищем как сервопривод постоянного вращения (CRServo)
        if (servo == null) {
            for (String name : possibleNames) {
                try {
                    crServo = hardwareMap.get(CRServo.class, name);
                    if (crServo != null) {
                        attachedName = name + " (CR)";
                        break;
                    }
                } catch (Exception ignored) {
                }
            }
        }

        // Начальное положение — опущена
        setDown();
    }

    /**
     * Проверяет, найден ли сервопривод в hardwareMap.
     */
    public boolean isAttached() {
        return servo != null || crServo != null;
    }

    /**
     * Возвращает имя найденного сервопривода из конфигурации.
     */
    public String getAttachedName() {
        return attachedName;
    }

    /**
     * Поднимает рампу (позиция OPEN / 1.0 -> 300°).
     */
    public void setUp() {
        if (servo != null) {
            servo.setPosition(RobotConstants.RAMP_POS_OPEN);
        } else if (crServo != null) {
            crServo.setPower(1.0);
        }
    }

    /**
     * Опускает рампу (позиция CLOSED / 0.0 -> 0°).
     */
    public void setDown() {
        if (servo != null) {
            servo.setPosition(RobotConstants.RAMP_POS_CLOSED);
        } else if (crServo != null) {
            crServo.setPower(0.0);
        }
    }

    /**
     * Устанавливает прямую позицию сервопривода (от 0.0 до 1.0).
     */
    public void setPosition(double position) {
        double clamped = Range.clip(position, 0.0, 1.0);
        if (servo != null) {
            servo.setPosition(clamped);
        } else if (crServo != null) {
            crServo.setPower(clamped > 0.1 ? 1.0 : (clamped < -0.1 ? -1.0 : 0.0));
        }
    }

    /**
     * Устанавливает угол в градусах для 300° сервопривода goBILDA (0° - 300°).
     */
    public void setDegrees(double degrees) {
        setPosition(degrees / 300.0);
    }

    /**
     * Возвращает текущую позицию (0.0 - 1.0) или мощность (для CRServo).
     */
    public double getPosition() {
        if (servo != null) {
            return servo.getPosition();
        } else if (crServo != null) {
            return crServo.getPower();
        }
        return 0.0;
    }

    /**
     * Возвращает угол в градусах (до 300°).
     */
    public double getDegrees() {
        return getPosition() * 300.0;
    }
}
