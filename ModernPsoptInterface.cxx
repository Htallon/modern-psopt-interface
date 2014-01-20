/**
 * This file is part of ModernPsoptInterface.
 *
 * Copyright 2014 Markus Sauermann
 *
 * ModernPsoptInterface is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * ModernPsoptInterface is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ModernPsoptInterface.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#include "ModernPsoptInterface.h"

namespace ModernPsoptInterface {


// CONSTRUCTOR

ModernPsoptInterface::ModernPsoptInterface()
{
}

// DEFAULT FUNCTIONS

void ModernPsoptInterface::SetDaeInitFunction(std::function<void (adouble* derivatives, adouble* path, adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase)> daeFunction)
{
	daeInitFunction = daeFunction;
}

void ModernPsoptInterface::SetEventInitFunction(std::function<void (adouble* e, adouble* initial_states, adouble* final_states, adouble* parameters, adouble& t0, adouble& tf, adouble* xad, int iphase)> eventFunction)
{
	eventInitFunction = eventFunction;
}

void ModernPsoptInterface::SetLinkagesInitFunction(std::function<void (adouble* linkages, adouble* xad)> linkageFunction)
{
	linkagesInitFunction = linkageFunction;
}

void ModernPsoptInterface::SetEndpointCostFunction(std::function<adouble (adouble* initial_states, adouble* final_states, adouble* parameters,adouble& t0, adouble& tf, adouble* xad, int iphase)> ecFunction)
{
	endpointCostFunction = ecFunction;
}
void ModernPsoptInterface::SetIntegrandCostFunction(std::function<adouble (adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase)> icFunction)
{
	integrandCostFunction = icFunction;
}

// HANDLING STATES AND DERIVATIVES

int ModernPsoptInterface::CreateNewState(int phase, double lower, double upper)
{
	if (phase < 1) {
		throw InputException("Phase in CreateNewState must be > 0.");

	}
	enlargeStateContainer(phase);
	StateStruct stSt;
	stSt.lowerBound = lower;
	stSt.upperBound = upper;
	stateContainer[phase-1].p.push_back(stSt);
	return stateContainer[phase-1].p.size()-1;
}
int ModernPsoptInterface::CreateNewState(int phase, double lower, double upper, GuessStruct guess)
{
	int state = CreateNewState(phase, lower, upper);
	_AddStateGuess(phase, state, guess);
	return state;
}
int ModernPsoptInterface::CreateNewState(int phase, double lower, double upper, GuessStruct guess, std::function<adouble()> derivative)
{
	int state = CreateNewState(phase, lower, upper);
	_AddStateDerivative(phase, state, derivative);
	_AddStateGuess(phase, state, guess);
	return state;
}
int ModernPsoptInterface::CreateNewState(int phase, double lower, double upper, std::function<adouble(adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase)> derivative)
{
	int state = CreateNewState(phase, lower, upper);
	_AddStateDerivative(phase, state, derivative);
	return state;

}
int ModernPsoptInterface::CreateNewState(int phase, double lower, double upper, GuessStruct guess, std::function<adouble(adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase)> derivative)
{
	int state = CreateNewState(phase, lower, upper);
	_AddStateDerivative(phase, state, derivative);
	_AddStateGuess(phase, state, guess);
	return state;
}

void ModernPsoptInterface::AddStateDerivative(int phase, int state, std::function<adouble(adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase)> derivative)
{
	verifyStateContainerSize(phase, state);
	_AddStateDerivative(phase, state, derivative);
}
void ModernPsoptInterface::AddStateGuess(int phase, int state, GuessStruct guess)
{
	verifyStateContainerSize(phase, state);
	_AddStateGuess(phase, state, guess);
}
/**
 * phase > 0, state >= 0
 */
void ModernPsoptInterface::_AddStateDerivative(int phase, int state, std::function<adouble()> derivative)
{
	stateContainer[phase-1].p[state].functionType = FunNoArgs;
	stateContainer[phase-1].p[state].derivationFunctionNoArgs = derivative;
}
/**
 * phase > 0, state >= 0
 */
void ModernPsoptInterface::_AddStateDerivative(int phase, int state, std::function<adouble(adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase)> derivative)
{
	stateContainer[phase-1].p[state].functionType = FunWithArgs;
	stateContainer[phase-1].p[state].derivationFunctionWithArgs = derivative;
}
/**
 * phase > 0, state >= 0
 */
void ModernPsoptInterface::_AddStateGuess(int phase, int state, GuessStruct guess)
{
	verifyStateContainerSize(phase, state);
	stateContainer[phase-1].guessMade = true;
	stateContainer[phase-1].p[state].guess = guess;
}
/**
 * phase > 0
 */
void ModernPsoptInterface::enlargeStateContainer(int phase)
{
	if ((int)stateContainer.size() < phase) {
		stateContainer.resize(phase);
		cout << "resizing state container to phases " << phase <<endl;
	}
}
/**
 * phase > 0, state >= 0
 */
void ModernPsoptInterface::verifyStateContainerSize(int phase, int state)
{
	int len = stateContainer.size();
	char buffer [100];
	if (phase < 1 || phase > len) {
		snprintf(buffer, 100, "StateContainer: Index for phase out of range : 1 <= %i <= %i", phase, len);
		throw InputException(buffer);
	}
	len = stateContainer[phase-1].p.size();
	if (state < 0 || state >= len) {
		snprintf(buffer, 100, "StateContainer: Index for state in phase %i out of range : 0 <= %i < %i", phase, state, len);
		throw InputException(buffer);
	}
}

// HANDLING CONTROLS

/**
 * phase > 0
 */
int ModernPsoptInterface::CreateNewControl(int phase, double lower, double upper)
{
	if (phase < 1) {
		throw InputException("Phase in CreateNewControl must be > 0.");

	}
	enlargeControlContainer(phase);
	controlStruct stSt;
	stSt.lowerBound = lower;
	stSt.upperBound = upper;
	controlContainer[phase-1].p.push_back(stSt);
	return controlContainer[phase-1].p.size()-1;
}

/**
 * phase > 0
 */
int ModernPsoptInterface::CreateNewControl(int phase, double lower, double upper, GuessStruct guess)
{
	int control = CreateNewControl(phase, lower, upper);
	_AddControlGuess(phase, control, guess);
	return control;
}

/**
 * phase > 0, control >= 0
 */
void ModernPsoptInterface::_AddControlGuess(int phase, int control, GuessStruct guess)
{
	verifyControlContainerSize(phase, control);
	controlContainer[phase-1].guessMade = true;
	controlContainer[phase-1].p[control].guess = guess;
}

/**
 * phase > 0
 */
void ModernPsoptInterface::enlargeControlContainer(int phase)
{
	if ((int)controlContainer.size() < phase) {
		controlContainer.resize(phase);
		cout << "resizing control container to phases " << phase << endl;
	}
}
/**
 * phase > 0, control >= 0
 */
void ModernPsoptInterface::verifyControlContainerSize(int phase, int control)
{
	int len = controlContainer.size();
	char buffer [100];
	if (phase < 1 || phase > len) {
		string x = "as ";
		x += phase + "add" + len;
		snprintf(buffer, 100, "ControlContainer: Index for phase out of range : 1 <= %i <= %i", phase, len);
		throw InputException(buffer);
	}
	len = controlContainer[phase-1].p.size();
	if (control < 0 || control >= len) {
		snprintf(buffer, 100, "ControlContainer: Index for control in phase %i out of range : 0 <= %i < %i", phase, control, len);
		throw InputException(buffer);
	}
}

// HANDLING EVENTS

/**
 * phase > 0
 */
int ModernPsoptInterface::CreateNewEvent(int phase, double lower, double upper)
{
	if (phase < 1) {
		throw InputException("Phase in CreateNewEvent must be > 0.");
	}
	enlargeEventContainer(phase);
	eventStruct eSt;
	eSt.lowerBound = lower;
	eSt.upperBound = upper;
	eventContainer[phase-1].push_back(eSt);
	return eventContainer[phase-1].size()-1;
}
/**
 * phase > 0
 */
int ModernPsoptInterface::CreateNewEvent(int phase, double lower, double upper, std::function<adouble(adouble* initial_states, adouble* final_states, adouble* parameters, adouble& t0, adouble& tf, adouble* xad)> eventFunction)
{
	int event = CreateNewEvent(phase, lower, upper);
	eventContainer[phase-1][event].functionType = FunWithArgs;
	eventContainer[phase-1][event].eventFunctionWithArgs = eventFunction;
	return event;
}

/**
 * phase > 0
 */
void ModernPsoptInterface::enlargeEventContainer(int phase)
{
	if ((int)eventContainer.size() < phase) {
		eventContainer.resize(phase);
			cout << "resizing event container to phases " << phase << endl;
	}
}

// HANDLING PATHS

int ModernPsoptInterface::CreateNewPath(int phase, double lower, double upper)
{
	if (phase < 1) {
			throw InputException("Phase in CreateNewPath must be > 0.");
		}
		enlargePathContainer(phase);
		pathStruct pSt;
		pSt.lowerBound = lower;
		pSt.upperBound = upper;
		pathContainer[phase-1].push_back(pSt);
		return pathContainer[phase-1].size()-1;
}

int ModernPsoptInterface::CreateNewPath(int phase, double lower, double upper, std::function<adouble(adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad)> pathFunction)
{
	int path = CreateNewPath(phase, lower, upper);
	pathContainer[phase-1][path].functionType = FunWithArgs;
	pathContainer[phase-1][path].pathFunctionWithArgs = pathFunction;
	return path;
}
/**
 * phase > 0
 */
void ModernPsoptInterface::enlargePathContainer(int phase)
{
	if ((int)pathContainer.size() < phase) {
		pathContainer.resize(phase);
			cout << "resizing path container to phases " << phase << endl;
	}
}


// HANDLING TIME

/**
 * phase > 0
 */
void ModernPsoptInterface::SetTimeConstraints(int phase, double beginLower, double beginUpper, double endLower, double endUpper)
{
	if (phase < 1) {
		throw InputException("Phase in SetTimeConstraints must be >= 0.");
	}

	enlargeTimeContainer(phase);

	timeContainer[phase-1].beginLower = beginLower;
	timeContainer[phase-1].beginUpper= beginUpper;
	timeContainer[phase-1].endLower = endLower;
	timeContainer[phase-1].endUpper = endUpper;
}

/**
 * phase > 0
 */
void ModernPsoptInterface::SetTimeConstraints(int phase, double beginLower, double beginUpper, double endLower, double endUpper, GuessStruct guess)
{
	SetTimeConstraints(phase, beginLower, beginUpper, endLower, endUpper);

	timeContainer[phase-1].guessMade = true;
	timeContainer[phase-1].guess = guess;
}

/**
 * phase > 0
 */
void ModernPsoptInterface::enlargeTimeContainer(int phase)
{
	if ((int)timeContainer.size() < phase) {
		timeContainer.resize(phase);
			cout << "resizing timer container to phases " << phase << endl;
	}
}

// MAIN FUNCTION

/**
 * linkages will be removed
 */
void ModernPsoptInterface::Psopt(string name, string outfilename, int nlinkages, AlgoConfigStruct algoConfig)
{
	nodesConfig = algoConfig.nodes;
	Level1Setup(name, outfilename, nlinkages);
	Level2Setup();
	Level3Setup();
	AlgoSetup(algoConfig);
	FunctionSetup();
	prePsoptFunction();
	psopt(solution, problem, algorithm);
}
void ModernPsoptInterface::prePsoptFunction()
{
	if (!display) return;
	cout << "linkages " << problem.nlinkages << endl;
	cout << "phases " << problem.nphases << endl;
	for (int i=1;i<= problem.nphases; i++) {
		cout << "Phase " << i << endl;
		problem.phase[i-1].nodes.Print("Nodes");
		if (displayTimes) {
			cout << problem.phase[i-1].bounds.lower.StartTime << " < Time Begin < " << problem.phase[i-1].bounds.upper.StartTime << endl;
			cout << problem.phase[i-1].bounds.lower.EndTime << " < Time End < " << problem.phase[i-1].bounds.upper.EndTime << endl;
			problem.phase[i-1].guess.time.Print("Time Guess");
		}
		cout << "States " << problem.phase[i-1].nstates << endl;
		if (displayStates) {
			problem.phase[i-1].bounds.lower.states.Print("Lower States");
			problem.phase[i-1].bounds.upper.states.Print("Upper States");
			problem.phase[i-1].guess.states.Print("State Guess");
		}
		cout << "Controls " << problem.phase[i-1].ncontrols << endl;
		if (displayControls) {
			problem.phase[i-1].bounds.lower.controls.Print("Lower Controls");
			problem.phase[i-1].bounds.upper.controls.Print("Upper Controls");
			problem.phase[i-1].guess.controls.Print("Control Guess");
		}
		cout << "Events " << problem.phase[i-1].nevents << endl;
		if (displayEvents) {
			problem.phase[i-1].bounds.lower.events.Print("Lower Events");
			problem.phase[i-1].bounds.upper.events.Print("Upper Events");
		}
		cout << "Paths " << problem.phase[i-1].npath << endl;
		if (displayPaths) {
			problem.phase[i-1].bounds.lower.path.Print("Lower Path");
			problem.phase[i-1].bounds.upper.path.Print("Upper Path");
		}
	}
}

void ModernPsoptInterface::Level1Setup(string name, string outfilename, int nlinkages)
{
	problem.name = name;
	problem.outfilename = outfilename;
	problem.nphases = max(stateContainer.size(), controlContainer.size());
	problem.nlinkages = nlinkages;
	psopt_level1_setup(problem);
}
void ModernPsoptInterface::Level2Setup()
{
	for (int phase = 1; phase <= problem.nphases; phase++)
	{
		Level2SetupPhase(phase);
	}
	psopt_level2_setup(problem, algorithm);
}
void ModernPsoptInterface::Level2SetupPhase(int phase)
{
	if (phase <= (int)stateContainer.size()) {
		problem.phases(phase).nstates = stateContainer[phase-1].p.size();
		cout << "setting states in phase " << phase << " to " << stateContainer[phase-1].p.size() << endl;
	} else {
		string err = "There are no states in phase ";
		err += phase;
		throw ModernPsoptInterfaceException(err);
	}
	if (phase <= (int)controlContainer.size()) {
		problem.phases(phase).ncontrols = controlContainer[phase-1].p.size();
		cout << "setting controls in phase " << phase << " to " << controlContainer[phase-1].p.size() << endl;
	}
	if (phase > (int)timeContainer.size()) {
		string err = "There are no time bounds in phase ";
		err += phase;
		throw ModernPsoptInterfaceException(err);
	}
	if (phase <= (int)eventContainer.size()) {
		problem.phases(phase).nevents = eventContainer[phase-1].size();
		cout << "setting events in phase " << phase << " to " << eventContainer[phase-1].size() << endl;
	} else {
		problem.phases(phase).nevents = 0;
	}
		if (phase <= (int)pathContainer.size()) {
			problem.phases(phase).npath = pathContainer[phase-1].size();
		}
	problem.phases(phase).nodes = nodesConfig.c_str();
}
void ModernPsoptInterface::Level3Setup()
{
	for (int phase = 1; phase <= problem.nphases; phase++)
	{
		int statesInPhase = stateContainer[phase-1].p.size();
		int controlsInPhase;
		try {
			controlsInPhase = controlContainer[phase-1].p.size();
		} catch (const out_of_range& e) {
			controlsInPhase = 0;
		}
		int eventsInPhase;
		try {
			eventsInPhase = eventContainer.at(phase-1).size();
		} catch (const out_of_range& e) {
			eventsInPhase = 0;
		}
		int pathsInPhase;
		try {
			pathsInPhase = pathContainer.at(phase-1).size();
		} catch (const out_of_range& e) {
			pathsInPhase = 0;
		}
		int nodesInPhase = problem.phases(phase).nodes(1,1);
		cout << "nodes in phase " << phase << " : " << nodesInPhase;
		// State Constraints
		for (int state = 1; state <= statesInPhase; state++) {
			problem.phases(phase).bounds.lower.states(state) = stateContainer[phase-1].p[state-1].lowerBound;
			problem.phases(phase).bounds.upper.states(state) = stateContainer[phase-1].p[state-1].upperBound;
//			cout << "state constraint " << state << " lo: " << problem.phases(phase).bounds.lower.states(state) << endl;
//			cout << "state constraint " << state << " up: " << problem.phases(phase).bounds.upper.states(state) << endl;
		}
		// State Guesses
		if (stateContainer[phase-1].guessMade) {
			DMatrix xGuess = zeros(statesInPhase, nodesInPhase);
			for (int state = 1; state <= statesInPhase; state++) {
				if (stateContainer[phase-1].p[state-1].guess.type != None)
				{
					xGuess(state, colon()) = GuessToDMatrix(stateContainer[phase-1].p[state-1].guess, nodesInPhase);
				}
			}
			problem.phases(phase).guess.states = xGuess;
		}
		// Control Constraints
		for (int control = 1; control <= controlsInPhase; control++) {
			problem.phases(phase).bounds.lower.controls(control) = controlContainer[phase-1].p[control-1].lowerBound;
			problem.phases(phase).bounds.upper.controls(control) = controlContainer[phase-1].p[control-1].upperBound;
//			cout << "control constraint " << control << " lo: " << problem.phases(phase).bounds.lower.controls(control) << endl;
//			cout << "control constraint " << control << " up: " << problem.phases(phase).bounds.upper.controls(control) << endl;
		}
		// Control Guesses
		if (controlContainer[phase-1].guessMade) {
			DMatrix uGuess = zeros(controlsInPhase, nodesInPhase);
			for (int control = 1; control <= controlsInPhase; control++) {
				if (controlContainer[phase-1].p[control-1].guess.type != None) {
					uGuess(control, colon()) = GuessToDMatrix(controlContainer[phase-1].p[control-1].guess, nodesInPhase);
				}
			}
			problem.phases(phase).guess.controls = uGuess;
		}
		// Event Constraints
		for (int event = 1; event <= eventsInPhase; event++) {
			problem.phases(phase).bounds.lower.events(event) = eventContainer[phase-1][event-1].lowerBound;
			problem.phases(phase).bounds.upper.events(event) = eventContainer[phase-1][event-1].upperBound;
//			cout << "event constraint " << event << " lo: " << problem.phases(phase).bounds.lower.events(event) << endl;
//			cout << "event constraint " << event << " up: " << problem.phases(phase).bounds.upper.events(event) << endl;
		}
		// Path Constraints
		for (int path = 1; path <= pathsInPhase; path++) {
			problem.phases(phase).bounds.lower.path(path) = pathContainer[phase-1][path-1].lowerBound;
			problem.phases(phase).bounds.upper.path(path) = pathContainer[phase-1][path-1].upperBound;

		}
		// Time Constraints
		problem.phases(phase).bounds.lower.StartTime = timeContainer[phase-1].beginLower;
		problem.phases(phase).bounds.upper.StartTime = timeContainer[phase-1].beginUpper;
		problem.phases(phase).bounds.lower.EndTime = timeContainer[phase-1].endLower;
		problem.phases(phase).bounds.upper.EndTime = timeContainer[phase-1].endUpper;
		if (timeContainer[phase-1].guessMade) {
			problem.phases(phase).guess.time = GuessToDMatrix(timeContainer[phase-1].guess, nodesInPhase);
		} else {
			problem.phases(phase).guess.time = linspace(
					(timeContainer[phase-1].beginUpper + timeContainer[phase-1].beginLower) / 2,
					(timeContainer[phase-1].endUpper >= INF) ? timeContainer[phase-1].endLower : (timeContainer[phase-1].endUpper + timeContainer[phase-1].endLower)/2,
							nodesInPhase);
		}
	}
}

DMatrix ModernPsoptInterface::GuessToDMatrix(GuessStruct guess, int nodes)
{
	if (guess.type == Linear) {
		return linspace(guess.linear.begin, guess.linear.end, nodes);
	} else if (guess.type == Circular) {
		DMatrix res = linspace (0, 0, nodes);
		double step = (guess.circular.end - guess.circular.begin) / (nodes - 1);
		double radiusStep = (guess.circular.radiusEnd - guess.circular.radiusBegin) / (nodes - 1);
		for (int i = 1; i<= nodes;i++) {
			res(i) = (guess.circular.radiusBegin + radiusStep * (i-1)) * sin(guess.circular.begin + step * (i-1));
		}
		return res;
	}
	throw NoGuessException("No Guess Available");
}
void ModernPsoptInterface::AlgoSetup(AlgoConfigStruct algoConfig)
{
	algorithm.derivatives = algoConfig.derivatives;
	algorithm.nlp_iter_max = algoConfig.nlpIterMax;
	algorithm.nlp_method = algoConfig.nlpMethod;
	algorithm.ode_tolerance = algoConfig.odeTolerance;
	algorithm.scaling = algoConfig.scaling;
	algorithm.nlp_tolerance = algoConfig.nlpTolerance;
}
void ModernPsoptInterface::FunctionSetup()
{
	problem.dae = [this] (adouble* derivatives, adouble* path, adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase) -> void
	{
		// init function
		if (daeInitFunction) {
			daeInitFunction(derivatives, path, states, controls, parameters, time, xad, iphase);
		}
		// derivatives
		for (int state = 0;state < problem.phases(iphase).nstates; state++) {
			if (stateContainer[iphase-1].p[state].functionType == FunNoArgs) {
				//cout << "no";
				derivatives[state] = stateContainer[iphase-1].p[state].derivationFunctionNoArgs();
			} else if (stateContainer[iphase-1].p[state].functionType == FunWithArgs) {
				derivatives[state] = stateContainer[iphase-1].p[state].derivationFunctionWithArgs(states, controls, parameters, time, xad, iphase);
				//cout << "wi" << derivatives[state];
			}
		}
		// paths
		for (int path = 0;path < problem.phases(iphase).npath;path++) {
			if (pathContainer[iphase-1][path].functionType == FunWithArgs) {
				pathContainer[iphase-1][path].pathFunctionWithArgs(states, controls, parameters, time, xad);
			} else if (pathContainer[iphase-1][path].functionType == FunNoArgs) {
				throw ModernPsoptInterfaceException("Unimplemented phasesNoArgsFunction");
			}
		}
	};
	problem.linkages = [this] (adouble* linkages, adouble* xad) -> void
	{
		if (linkagesInitFunction) {
			linkagesInitFunction(linkages, xad);
		}
	};
	problem.events = [this] (adouble* e, adouble* initial_states, adouble* final_states, adouble* parameters, adouble& t0, adouble& tf, adouble* xad, int iphase) -> void
	{
		if (eventInitFunction) {
			eventInitFunction(e, initial_states, final_states, parameters, t0, tf, xad, iphase);
		}
		for (int event = 0; event< problem.phases(iphase).nevents; event++) {
			if (eventContainer[iphase-1][event].functionType == FunNoArgs) {
				e[event] = eventContainer[iphase-1][event].eventFunctionNoArgs();
			} else if (eventContainer[iphase-1][event].functionType == FunWithArgs) {
				e[event] = eventContainer[iphase-1][event].eventFunctionWithArgs(initial_states, final_states, parameters, t0, tf, xad);
			}
		}
	};
	if (endpointCostFunction) {
		problem.endpoint_cost = [this] (adouble* initial_states, adouble* final_states, adouble* parameters, adouble& t0, adouble& tf, adouble* xad, int iphase) -> adouble
				{
			return endpointCostFunction(initial_states, final_states, parameters, t0, tf, xad, iphase);
				};
	};
	if (integrandCostFunction) {
		problem.integrand_cost = [this] (adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase) -> adouble
				{
			return integrandCostFunction(states, controls, parameters, time, xad, iphase);
				};
	};
}

void ModernPsoptInterface::plot_win(DMatrix& theta, DMatrix& r, string title, string xlabel, string ylabel, string legend, string terminal, string output, bool show)
{
	plot(theta, r, const_cast<char *>(title.c_str()), const_cast<char *>(xlabel.c_str()), const_cast<char *>(ylabel.c_str()), const_cast<char *>(legend.c_str()), const_cast<char *>(terminal.c_str()), const_cast<char *>(output.c_str()));
	if (show) {
		string com = "explorer.exe " + output;
		system(com.c_str());
	}
}

ResultVectors ModernPsoptInterface::getResults()
{
	DMatrix u = solution.get_controls_in_phase(1);
	DMatrix x = solution.get_states_in_phase  (1);
	DMatrix t = solution.get_time_in_phase    (1);

	for (int i = 2; i <=problem.nphases;i++) {
		u = u || solution.get_controls_in_phase(i);
		x = x || solution.get_states_in_phase  (i);
		t = t || solution.get_time_in_phase    (i);
	}
	ResultVectors res;
	res.t = t;
	res.x = x;
	res.u = u;
	return res;
}

} // end namespace
