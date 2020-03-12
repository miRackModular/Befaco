#include <string.h>
#include "plugin.hpp"
#include "pffft.h"


float *springReverbIR = NULL;
int springReverbIRLen = 0, springReverbIRSampleRate = 0;

void springReverbInit(int sampleRate) {
	if (springReverbIRSampleRate == sampleRate)
		return;

	std::string irFilename = asset::plugin(pluginInstance, string::f("res/SpringReverbIR%d.pcm", sampleRate));
	FILE *f = fopen(irFilename.c_str(), "rb");
	
	if (!f)
		return;

	fseek(f, 0, SEEK_END);
	int size = ftell(f);
	fseek(f, 0, SEEK_SET);

	springReverbIRLen = size / sizeof(float);
	springReverbIR = (float*)realloc(springReverbIR, size);
	fread(springReverbIR, sizeof(float), springReverbIRLen, f);
	fclose(f);

	// TODO Add springReverbDestroy() function once plugins have destroy() callbacks
}


static const size_t BLOCK_SIZE = 1024;


struct SpringReverb : Module {
	enum ParamIds {
		WET_PARAM,
		LEVEL1_PARAM,
		LEVEL2_PARAM,
		HPF_PARAM,
		NUM_PARAMS
	};
	enum InputIds {
		CV1_INPUT,
		CV2_INPUT,
		IN1_INPUT,
		IN2_INPUT,
		MIX_CV_INPUT,
		NUM_INPUTS
	};
	enum OutputIds {
		MIX_OUTPUT,
		WET_OUTPUT,
		NUM_OUTPUTS
	};
	// enum LightIds {
	// 	PEAK_LIGHT,
	// 	VU1_LIGHT,
	// 	NUM_LIGHTS = VU1_LIGHT + 7
	// };

	dsp::RealTimeConvolver *convolver = NULL;
	dsp::DoubleRingBuffer<float, 16*BLOCK_SIZE> inputBuffer;
	dsp::DoubleRingBuffer<float, 16*BLOCK_SIZE> outputBuffer;

	dsp::RCFilter dryFilter;
	// dsp::PeakFilter vuFilter;
	// dsp::PeakFilter lightFilter;

	float level1 = 0, level2 = 0;
	float p1 = -1, p2 = -1, cv1 = -1, cv2 = -1, hpf = -1;	

	SpringReverb() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, 0);
		configParam(WET_PARAM, 0.0, 1.0, 0.5, "Dry/wet", "%", 0, 100);
		configParam(LEVEL1_PARAM, 0.0, 1.0, 0.0, "In 1 level", "%", 0, 100);
		configParam(LEVEL2_PARAM, 0.0, 1.0, 0.0, "In 1 level", "%", 0, 100);
		configParam(HPF_PARAM, 0.0, 1.0, 0.5, "High pass filter cutoff");

		springReverbInit(api0::engineGetSampleRate());

		convolver = new dsp::RealTimeConvolver(BLOCK_SIZE);
		convolver->setKernel(springReverbIR, springReverbIRLen);
	}

	void onSampleRateChange() override {
		springReverbInit(api0::engineGetSampleRate());
		convolver->setKernel(springReverbIR, springReverbIRLen);
	}

	~SpringReverb() {
		delete convolver;
	}

	void process(const ProcessArgs &args) override {
		float in1 = inputs[IN1_INPUT].getVoltage();
		float in2 = inputs[IN2_INPUT].getVoltage();

		const float levelScale = 0.030;
		const float levelBase = 25.0;
		if (p1 != params[LEVEL1_PARAM].value || cv1 != inputs[CV1_INPUT].value)
		{
			p1 = params[LEVEL1_PARAM].value;
			cv1 = inputs[CV1_INPUT].value;
			level1 = levelScale * dsp::exponentialBipolar(levelBase, params[LEVEL1_PARAM].getValue()) * inputs[CV1_INPUT].getNormalVoltage(10.0) / 10.0;
		}
		if (p2 != params[LEVEL2_PARAM].value || cv2 != inputs[CV2_INPUT].value)
		{
			p2 = params[LEVEL2_PARAM].value;
			cv2 = inputs[CV2_INPUT].value;
			level2 = levelScale * dsp::exponentialBipolar(levelBase, params[LEVEL2_PARAM].getValue()) * inputs[CV2_INPUT].getNormalVoltage(10.0) / 10.0;
		}

		float dry = in1 * level1 + in2 * level2;

		// HPF on dry
		if (hpf != params[HPF_PARAM].value)
		{
			hpf = params[HPF_PARAM].value;
			float dryCutoff = 200.0 * std::pow(20.0, params[HPF_PARAM].getValue()) * args.sampleTime;
			dryFilter.setCutoff(dryCutoff);
		}
		dryFilter.process(dry);

		// Add dry to input buffer
		if (!inputBuffer.full())
			inputBuffer.push(dryFilter.highpass());


		if (outputBuffer.empty()) {
			// Convolve block
			convolver->processBlock(inputBuffer.startData(), outputBuffer.endData());
			inputBuffer.startIncr(BLOCK_SIZE);
			outputBuffer.endIncr(BLOCK_SIZE);
		}

		// Set output
		if (outputBuffer.empty())
			return;
		
		float wet = outputBuffer.shift();
		float balance = clamp(params[WET_PARAM].getValue() + inputs[MIX_CV_INPUT].getVoltage() / 10.0f, 0.0f, 1.0f);
		float mix = crossfade(in1, wet, balance);

		outputs[WET_OUTPUT].setVoltage(clamp(wet, -10.0f, 10.0f));
		outputs[MIX_OUTPUT].setVoltage(clamp(mix, -10.0f, 10.0f));

		// // Set lights
		// float lightRate = 5.0 * args.sampleTime;
		// vuFilter.setRate(lightRate);
		// vuFilter.process(std::fabs(wet));
		// lightFilter.setRate(lightRate);
		// lightFilter.process(std::fabs(dry*50.0));

		// float vuValue = vuFilter.peak();
		// for (int i = 0; i < 7; i++) {
		// 	float light = std::pow(1.413, i) * vuValue / 10.0 - 1.0;
		// 	lights[VU1_LIGHT + i].value = clamp(light, 0.0f, 1.0f);
		// }
		// lights[PEAK_LIGHT].value = lightFilter.peak();
	}
};


struct SpringReverbWidget : ModuleWidget {
	SpringReverbWidget(SpringReverb *module) {
		setModule(module);
		setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/SpringReverb.svg")));

		addChild(createWidget<Knurlie>(Vec(15, 0)));
		addChild(createWidget<Knurlie>(Vec(15, 365)));
		addChild(createWidget<Knurlie>(Vec(15*6, 0)));
		addChild(createWidget<Knurlie>(Vec(15*6, 365)));

		addParam(createParam<BefacoBigKnob>(Vec(22, 29), module, SpringReverb::WET_PARAM));

		addParam(createParam<BefacoSlidePot>(Vec(12, 116), module, SpringReverb::LEVEL1_PARAM));
		addParam(createParam<BefacoSlidePot>(Vec(93, 116), module, SpringReverb::LEVEL2_PARAM));

		addParam(createParam<Davies1900hWhiteKnob>(Vec(42, 210), module, SpringReverb::HPF_PARAM));

		addInput(createInput<PJ301MPort>(Vec(7, 243), module, SpringReverb::CV1_INPUT));
		addInput(createInput<PJ301MPort>(Vec(88, 243), module, SpringReverb::CV2_INPUT));
		addInput(createInput<PJ301MPort>(Vec(27, 281), module, SpringReverb::IN1_INPUT));
		addInput(createInput<PJ301MPort>(Vec(67, 281), module, SpringReverb::IN2_INPUT));

		addOutput(createOutput<PJ301MPort>(Vec(7, 317), module, SpringReverb::MIX_OUTPUT));
		addInput(createInput<PJ301MPort>(Vec(47, 324), module, SpringReverb::MIX_CV_INPUT));
		addOutput(createOutput<PJ301MPort>(Vec(88, 317), module, SpringReverb::WET_OUTPUT));

		// addChild(createLight<MediumLight<GreenRedLight>>(Vec(55, 269), module, SpringReverb::PEAK_LIGHT));
		// addChild(createLight<MediumLight<RedLight>>(Vec(55, 113), module, SpringReverb::VU1_LIGHT + 0));
		// addChild(createLight<MediumLight<YellowLight>>(Vec(55, 126), module, SpringReverb::VU1_LIGHT + 1));
		// addChild(createLight<MediumLight<YellowLight>>(Vec(55, 138), module, SpringReverb::VU1_LIGHT + 2));
		// addChild(createLight<MediumLight<GreenLight>>(Vec(55, 150), module, SpringReverb::VU1_LIGHT + 3));
		// addChild(createLight<MediumLight<GreenLight>>(Vec(55, 163), module, SpringReverb::VU1_LIGHT + 4));
		// addChild(createLight<MediumLight<GreenLight>>(Vec(55, 175), module, SpringReverb::VU1_LIGHT + 5));
		// addChild(createLight<MediumLight<GreenLight>>(Vec(55, 188), module, SpringReverb::VU1_LIGHT + 6));
	}
};


Model *modelSpringReverb = createModel<SpringReverb, SpringReverbWidget>("SpringReverb");
