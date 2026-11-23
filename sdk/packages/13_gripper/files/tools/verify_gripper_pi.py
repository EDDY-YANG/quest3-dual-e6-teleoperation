"""Exercise actual Uno HEX through emulated UART, A0 and D2. No physical I/O."""
import argparse
import json
from pathlib import Path
from verify_gripper_firmware import EmulatedUart
from gripper_uno import PositionPI, PortSettings, UnoWire


def check_pulses(port, expected):
    pulses=port.request({"ms":110})["pulses"]
    assert len(pulses)>=2, pulses
    widths=[p["width_us"] for p in pulses[-2:]]
    assert all(abs(v-expected)<8 for v in widths), (expected,widths)
    return widths[-1]


def verify(direct_image, pi_image, module):
    # Fixture calibration and gains for arithmetic checks only.
    pi=PositionPI(100,900,0.,60.,120,50,60)
    cfg=PortSettings("AVR-SIM",400,1300,0,True,pi)
    uart=EmulatedUart(direct_image,module,2)
    try:
        direct=UnoWire(uart,"right",100)
        direct.identify()
        assert uart.request({"line":"ARMPI R 5 400 1300 100 900 120 50 60"})["reply"]=="ERR"
        direct.arm(PortSettings("AVR-SIM",400,1300,0,True))
        for u in (1300,400,1300):
            direct.set_pwm(u)
            check_pulses(uart,u)
    finally:
        uart.close()
    uart=EmulatedUart(pi_image,module,2)
    try:
        def adc(value): uart.request({"adc":value})
        wire=UnoWire(uart,"right",321)
        adc(500)
        wire.identify()
        assert uart.request({"line":"ADC?"})["reply"]=="ADC 500"
        wire.arm(cfg)
        assert not uart.request({"ms":50})["pulses"]
        wire.set_pwm(1300)
        adc(300)  # fixture measures 25% open; command 50%
        below=wire.set_pwm(850)
        assert below["measured_gap_mm"]==15 and below["applied_pwm_us"]==880, below
        for _ in range(30):
            uart.request({"ms":110})
            saturated=wire.set_pwm(850)["applied_pwm_us"]
        assert saturated==910
        adc(700)
        above=wire.set_pwm(850)["applied_pwm_us"]
        assert 810<=above<=820, above
        adc(500)  # jaw still half open despite a full-close command
        closed=wire.set_pwm(400)
        assert closed["applied_pwm_us"]==400 and closed["measured_gap_mm"]==30
        check_pulses(uart,400)
        opened=wire.set_pwm(1300)
        assert opened["applied_pwm_us"]==1300 and opened["measured_gap_mm"]==30
        acknowledged=uart.request({"ms":0})["now_us"]
        pulses=uart.request({"ms":70})["pulses"]
        first=next(p for p in pulses if p["at_us"]-p["width_us"]>=acknowledged)
        full_open_wave=first["width_us"]
        assert abs(full_open_wave-1300)<8 and first["at_us"]-acknowledged<22500
        assert wire.set_pwm(850)["applied_pwm_us"]==850  # no retained integral
        adc(0)
        uart.request({"ms":45})
        assert not uart.request({"ms":70})["pulses"]
        assert uart.request({"line":"SET 321 100 850"})["reply"]=="ERR"
        adc(700)
        reverse=PositionPI(900,100,0.,60.,120,50,60)
        wire=UnoWire(uart,"right",322)
        wire.arm(PortSettings("AVR-SIM",1300,400,0,True,reverse))
        wire.set_pwm(400)
        assert wire.set_pwm(850)["applied_pwm_us"]==820  # reversed ADC and PWM
        uart.request({"ms":360})
        assert not uart.request({"ms":70})["pulses"]
        return {"scope":"simulated Uno D2/A0; no physical jaw measurement",
                "under_open_pwm_us":below["applied_pwm_us"],"over_open_pwm_us":above,
                "limited_pwm_us":saturated,"reversal_command_us":opened["applied_pwm_us"],
                "reversal_gpio_us":full_open_wave,"sensor_fault_and_watchdog_disabled_pwm":True}
    finally:
        uart.close()


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--direct",type=Path,required=True)
    p.add_argument("--pi",type=Path,required=True)
    p.add_argument("--avr8js-path",type=Path,required=True)
    args=p.parse_args()
    print(json.dumps(verify(args.direct,args.pi,args.avr8js_path),indent=2))
