#include "motor.hpp"

Motor::Motor(const uint32_t frequency, const uint gpio) {
	this->frequency = frequency;
	this->gpio = gpio;
	this->slice_num = pwm_gpio_to_slice_num(gpio);
	this->pwm_channel = pwm_gpio_to_channel(gpio);
}

Motor::~Motor() = default;

void Motor::pwm_set_freq() {
	//Value that allows to minimize the frequency of pwm bellow 1.9khz (minimal default value)
	uint32_t divider16 = RASP_CLOCK / this->frequency / 4096 +
											 (RASP_CLOCK % (this->frequency * 4096) != 0);

	if (divider16 / 16 == 0)
		divider16 = 16;

	//This is the highest value the counter will count up to before either rolling over to zero
	//or starting to count down to zero
	this->wrap = RASP_CLOCK * 16 / divider16 / frequency - 1;

	printf("Iniciando motor");

	pwm_set_clkdiv_int_frac(this->slice_num, divider16 / 16,
													divider16 & 0xF);
	pwm_set_wrap(this->slice_num, this->wrap);

	printf("Motor iniciado com wrap: %d e divider16: %d\n", this->wrap, divider16);
}

void Motor::vInitMotors() {
	gpio_set_function(this->gpio, GPIO_FUNC_PWM);

	this->pwm_set_freq();

	this->setSpeed(0);

	pwm_set_enabled(this->slice_num, true);
}

void Motor::setSpeed(const uint speed) const {
	const float duty_min = 5.1f;  // Duty mínimo detectado (em %)
	const float duty_max = 14.4f; // Duty máximo detectado (em %)

	// Normalização para que speed (0-100) corresponda ao intervalo real (5.1% - 14.4%)
	float duty_cycle = ((speed / 100.0f) * (duty_max - duty_min)) + duty_min;

	// Converte para um valor inteiro válido para pwm_set_chan_level
	uint16_t duty_level = static_cast<uint16_t>((this->wrap * duty_cycle) / 100.0f + 0.5f);

	// Define o nível PWM com valor inteiro
	pwm_set_chan_level(this->slice_num, this->pwm_channel, duty_level);
}

uint Motor::get_slice_num() const {
	return  this->slice_num;
}
