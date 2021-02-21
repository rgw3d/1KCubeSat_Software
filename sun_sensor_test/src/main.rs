//#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate matrix_helper;
extern crate nb;
extern crate rtic;
extern crate stm32l0xx_hal as hal;
extern crate void;

use cortex_m_semihosting as _;
use hal::{prelude::*, serial};
use panic_semihosting as _;

#[rtic::app(device = hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        rx: serial::Rx<serial::USART1>,
        tx: serial::Tx<serial::USART1>,

        //rx_prod: spsc::Producer<'static, u8, U8>,
        //rx_cons: spsc::Consumer<'static, u8, U8>,
        //watchdog_done: WatchdogPinType,
        //led1: gpio::PB2<gpio::Output<gpio::OpenDrain>>,
        //led2: gpio::PB3<gpio::Output<gpio::OpenDrain>>,
        //led3: gpio::PB4<gpio::Output<gpio::OpenDrain>>,
        //led4: gpio::PB5<gpio::Output<gpio::OpenDrain>>,
        //led5: gpio::PB6<gpio::Output<gpio::OpenDrain>>,
        led1: hal::gpio::gpiob::PB9<hal::gpio::Output<hal::gpio::PushPull>>,
        led2: hal::gpio::gpiob::PB8<hal::gpio::Output<hal::gpio::PushPull>>,
        led3: hal::gpio::gpiob::PB5<hal::gpio::Output<hal::gpio::PushPull>>,
        led4: hal::gpio::gpiob::PB4<hal::gpio::Output<hal::gpio::PushPull>>,
        led5: hal::gpio::gpiob::PB7<hal::gpio::Output<hal::gpio::PushPull>>,
        led6: hal::gpio::gpiob::PB6<hal::gpio::Output<hal::gpio::PushPull>>,
        led7: hal::gpio::gpiob::PB3<hal::gpio::Output<hal::gpio::PushPull>>,
        led8: hal::gpio::gpioc::PC12<hal::gpio::Output<hal::gpio::PushPull>>,
        led9: hal::gpio::gpioc::PC10<hal::gpio::Output<hal::gpio::PushPull>>,
        led10: hal::gpio::gpiob::PB15<hal::gpio::Output<hal::gpio::PushPull>>,
        led11: hal::gpio::gpiob::PB12<hal::gpio::Output<hal::gpio::PushPull>>,
        led12: hal::gpio::gpiob::PB11<hal::gpio::Output<hal::gpio::PushPull>>,
        led13: hal::gpio::gpiob::PB14<hal::gpio::Output<hal::gpio::PushPull>>,
        led14: hal::gpio::gpiob::PB13<hal::gpio::Output<hal::gpio::PushPull>>,
        led15: hal::gpio::gpiob::PB10<hal::gpio::Output<hal::gpio::PushPull>>,
        led16: hal::gpio::gpiob::PB2<hal::gpio::Output<hal::gpio::PushPull>>,
        led17: hal::gpio::gpioc::PC11<hal::gpio::Output<hal::gpio::PushPull>>,
        s0: hal::gpio::gpioc::PC13<hal::gpio::Output<hal::gpio::PushPull>>,
        s1: hal::gpio::gpioc::PC14<hal::gpio::Output<hal::gpio::PushPull>>,
        s2: hal::gpio::gpioc::PC15<hal::gpio::Output<hal::gpio::PushPull>>,
        s3: hal::gpio::gpioh::PH0<hal::gpio::Output<hal::gpio::PushPull>>,
        en: hal::gpio::gpioh::PH1<hal::gpio::Output<hal::gpio::PushPull>>,
        a3: hal::gpio::gpioa::PA3<hal::gpio::Analog>,
        adc: hal::adc::Adc<hal::adc::Ready>,
    }

    #[init()]
    fn init(cx: init::Context) -> init::LateResources {
        //cortex_m::asm::bkpt();
        // Device specific peripherals
        let device: hal::pac::Peripherals = cx.device;

        // Constrain some device peripherials so we can setup the clock config below
        //let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.freeze(hal::rcc::Config::hsi16());
        //let mut pwr = device.PWR.constrain(&mut rcc.apb1r1);

        //// Setup clock config
        //let rcc_cfgr = rcc
        //    .cfgr
        //    .lsi(false)
        //    .lse(
        //        hal::rcc::CrystalBypass::Enable,
        //        hal::rcc::ClockSecuritySystem::Disable,
        //    )
        //    .hsi48(false)
        //    .msi(hal::rcc::MsiFreq::RANGE4M)
        //    //.sysclk(4.mhz());
        //    .hclk(4.mhz())
        //    .pclk1(4.mhz())
        //    .pclk2(4.mhz());

        //let rcc_reg = unsafe { &*hal::stm32::RCC::ptr() };
        ////let rcc_reg = unsafe { &*rcc::ptr() };
        //unsafe {
        //    rcc_reg.ccipr.write(|w| w.usart3sel().bits(0b11));
        //}

        //// Freeze the clock configuration
        //let clocks = rcc_cfgr.freeze(&mut flash.acr, &mut pwr);

        // Grab handles to GPIO banks
        let mut gpioa = device.GPIOA.split(&mut rcc);
        let mut gpiob = device.GPIOB.split(&mut rcc);
        let mut gpioc = device.GPIOC.split(&mut rcc);
        let mut gpioh = device.GPIOH.split(&mut rcc);
        let mut adc = device.ADC.constrain(&mut rcc);

        let mut led1 = gpiob.pb9.into_push_pull_output();
        let mut led2 = gpiob.pb8.into_push_pull_output();
        let mut led3 = gpiob.pb5.into_push_pull_output();
        let mut led4 = gpiob.pb4.into_push_pull_output();
        let mut led5 = gpiob.pb7.into_push_pull_output();
        let mut led6 = gpiob.pb6.into_push_pull_output();
        let mut led7 = gpiob.pb3.into_push_pull_output();
        let mut led8 = gpioc.pc12.into_push_pull_output();
        let mut led9 = gpioc.pc10.into_push_pull_output();
        let mut led10 = gpiob.pb15.into_push_pull_output();
        let mut led11 = gpiob.pb12.into_push_pull_output();
        let mut led12 = gpiob.pb11.into_push_pull_output();
        let mut led13 = gpiob.pb14.into_push_pull_output();
        let mut led14 = gpiob.pb13.into_push_pull_output();
        let mut led15 = gpiob.pb10.into_push_pull_output();
        let mut led16 = gpiob.pb2.into_push_pull_output();
        let mut led17 = gpioc.pc11.into_push_pull_output();
        let mut s0 = gpioc.pc13.into_push_pull_output();
        let mut s1 = gpioc.pc14.into_push_pull_output();
        let mut s2 = gpioc.pc15.into_push_pull_output();
        let mut s3 = gpioh.ph0.into_push_pull_output();
        let mut en = gpioh.ph1.into_push_pull_output();

        led17.set_high().ok();

        let mut a3 = gpioa.pa3.into_analog();
        let val: u16 = adc.read(&mut a3).unwrap();

        let mut tx_pin = gpioa.pa9;
        let mut rx_pin = gpioa.pa10;

        let serial = device
            .USART1
            .usart(tx_pin, rx_pin, serial::Config::default(), &mut rcc)
            .unwrap();
        let (mut tx, mut rx) = serial.split();

        //// Configure the USART3 pins
        //let tx_pin = gpiob.pb10.into_af7(&mut gpiob.moder, &mut gpiob.afrh);
        //let rx_pin = gpiob.pb11.into_af7(&mut gpiob.moder, &mut gpiob.afrh);

        //// Send a string over the UART
        //let sent = b"START\n\r";
        //for elem in sent {
        //    block!(tx.write(*elem)).ok();
        //}

        init::LateResources {
            rx,
            tx,
            //rx_prod,
            //rx_cons,
            //watchdog_done,
            led1,
            led2,
            led3,
            led4,
            led5,
            led6,
            led7,
            led8,
            led9,
            led10,
            led11,
            led12,
            led13,
            led14,
            led15,
            led16,
            led17,
            s0,
            s1,
            s2,
            s3,
            en,
            a3,
            adc,
        }
    }

    #[idle(resources = [tx, led1, led2, led3, led4, led5, led6, led7, led8, led9, led10, led11, led12, led13, led14, led15, led16, led17, s0, s1, s2, s3, en, a3, adc])]
    fn idle(mut cx: idle::Context) -> ! {
        //let rx_queue = cx.resources.rx_cons;
        let tx = cx.resources.tx;
        cx.resources.en.set_low().ok();
        cx.resources.s0.set_low().ok();
        cx.resources.s1.set_low().ok();
        cx.resources.s2.set_low().ok();
        cx.resources.s3.set_low().ok();

        // Maps such that shade position allows for best fit plane
        let led_array: [&mut dyn OutputPin<Error = void::Void>; 16] = [
            cx.resources.led7,
            cx.resources.led5,
            cx.resources.led8,
            cx.resources.led6,
            cx.resources.led3,
            cx.resources.led1,
            cx.resources.led4,
            cx.resources.led2,
            cx.resources.led11,
            cx.resources.led9,
            cx.resources.led12,
            cx.resources.led10,
            cx.resources.led15,
            cx.resources.led13,
            cx.resources.led16,
            cx.resources.led14,
        ];
        // Maps directly to LED's
        //let led_array: [&mut dyn OutputPin<Error = void::Void>; 16] = [
        //    cx.resources.led6,
        //    cx.resources.led8,
        //    cx.resources.led5,
        //    cx.resources.led7,
        //    cx.resources.led2,
        //    cx.resources.led4,
        //    cx.resources.led1,
        //    cx.resources.led3,
        //    cx.resources.led10,
        //    cx.resources.led12,
        //    cx.resources.led9,
        //    cx.resources.led11,
        //    cx.resources.led14,
        //    cx.resources.led16,
        //    cx.resources.led13,
        //    cx.resources.led15,
        //];

        let mut adc_vals: [f32; 16] = [0.0; 16];
        let mut matrix_a_out: [f32; 48] = [0.0; 48];
        {
            let mut matrix_a: [f32; 48] = [0.0; 48];
            let adc_coord_map: [(i32, i32); 16] = [
                (2, 2),
                (2, 1),
                (1, 2),
                (1, 1),
                //
                (-1, 2),
                (-1, 1),
                (-2, 2),
                (-2, 1),
                //
                (-1, -1),
                (-1, -2),
                (-2, -1),
                (-2, -2),
                //
                (2, -1),
                (2, 2),
                (1, -1),
                (1, -2),
            ];
            // populate matrix a
            for i in 0..16 {
                let (a, b) = adc_coord_map[i];
                matrix_a[(i * 3) + 0] = a as f32;
                matrix_a[(i * 3) + 1] = b as f32;
                matrix_a[(i * 3) + 2] = 1.0;
            }

            let mut matrix_a_t: [f32; 48] = [0.0; 48];
            matrix_helper::matrix_transpose(&matrix_a, &mut matrix_a_t, 16, 3);
            let mut matrix_tmp: [f32; 9] = [0.0; 9];
            matrix_helper::multiply(&matrix_a_t, &matrix_a, &mut matrix_tmp, 3, 16, 16, 3);
            let mut matrix_a_t_mul_inv: [f32; 9] = [0.0; 9];
            match matrix_helper::inverse_3x3(&matrix_tmp, &mut matrix_a_t_mul_inv) {
                Err(()) => panic!(),
                _ => (),
            }
            matrix_helper::multiply(
                &matrix_a_t_mul_inv,
                &matrix_a_t,
                &mut matrix_a_out,
                3,
                3,
                3,
                16,
            );
        }

        // fake pwm led driver garbage
        let mut cutoff: f32 = 0.0;
        const MAX_CUTOFF: f32 = 4096.0;
        const STEP: f32 = 400.0;
        let mut min_adc: f32 = 0.0;
        let mut max_adc: f32 = 0.0;
        loop {
            cutoff = (cutoff + STEP) % MAX_CUTOFF;
            min_adc = MAX_CUTOFF;
            max_adc = 0.0;
            for n in 0..16 {
                set_analog_mux(
                    n,
                    cx.resources.s0,
                    cx.resources.s1,
                    cx.resources.s2,
                    cx.resources.s3,
                );
                cortex_m::asm::delay(10);
                adc_vals[n as usize] = cx.resources.adc.read(cx.resources.a3).unwrap();

                // LED Driver Method 1
                //if adc_vals[n as usize] > cutoff {
                //    led_array[n as usize].set_high().ok();
                //} else {
                //    led_array[n as usize].set_low().ok();
                //}

                // LED Driver method 2
                //let itrs = (MAX_CUTOFF / STEP) as usize;
                //cutoff = 0.0;
                //for _ in 0..itrs {
                //    cutoff = (cutoff + STEP) % MAX_CUTOFF;
                //    for x in 0..16 {
                //        if adc_vals[x as usize] > cutoff {
                //            led_array[x as usize].set_high().ok();
                //        } else {
                //            led_array[x as usize].set_low().ok();
                //        }
                //    }
                //}

                // LED driver method 3
                if (adc_vals[n as usize] < min_adc) {
                    min_adc = adc_vals[n as usize];
                }
                if (adc_vals[n as usize] > max_adc) {
                    max_adc = adc_vals[n as usize];
                }
                let itrs = (MAX_CUTOFF / STEP) as usize;
                cutoff = min_adc;
                let step = (max_adc - min_adc) / 10.0;
                for _ in 0..10 {
                    cutoff = cutoff + step;
                    for x in 0..16 {
                        if adc_vals[x as usize] >= cutoff {
                            led_array[x as usize].set_high().ok();
                        } else {
                            led_array[x as usize].set_low().ok();
                        }
                    }
                }
            }
            //    cortex_m::asm::bkpt();
            // fit = (A.T * A).I * A.T * b
        }
        //    //if let Some(b) = rx_queue.dequeue() {
        //    //    //block!(tx.write(b)).unwrap();
        //    //}
        //}
    }

    //#[task(binds = USART3, resources = [rx, rx_prod, led5], priority = 3)]
    //fn usart3(cx: usart3::Context) {
    //    cx.resources.led5.set_high().ok();
    //    let rx = cx.resources.rx;
    //    let queue = cx.resources.rx_prod;

    //    let b = match rx.read() {
    //        Ok(b) => b,
    //        Err(_err) => b'x',
    //    };
    //    match queue.enqueue(b) {
    //        Ok(()) => (),
    //        Err(_err) => {}
    //    }
    //}

    //extern "C" {

    //    // I think any of the interupts work??
    //    //fn I2C2();
    //}
};

fn set_analog_mux(
    mux: u32,
    s0: &mut hal::gpio::gpioc::PC13<hal::gpio::Output<hal::gpio::PushPull>>,
    s1: &mut hal::gpio::gpioc::PC14<hal::gpio::Output<hal::gpio::PushPull>>,
    s2: &mut hal::gpio::gpioc::PC15<hal::gpio::Output<hal::gpio::PushPull>>,
    s3: &mut hal::gpio::gpioh::PH0<hal::gpio::Output<hal::gpio::PushPull>>,
) {
    if mux & 0x1 == 0x1 {
        s0.set_high().ok();
    } else {
        s0.set_low().ok();
    }

    if mux & 0x2 == 0x2 {
        s1.set_high().ok();
    } else {
        s1.set_low().ok();
    }

    if mux & 0x4 == 0x4 {
        s2.set_high().ok();
    } else {
        s2.set_low().ok();
    }

    if mux & 0x8 == 0x8 {
        s3.set_high().ok();
    } else {
        s3.set_low().ok();
    }
}