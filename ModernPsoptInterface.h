/*! \file ModernPsoptInterface.cxx
 *
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

#pragma once

#include <dmatrixv.h>
#include <adolc/adouble.h>

#include <vector>
#include <functional>
#include <memory>

namespace PsoptInterface {


class PsoptInterfaceException : public exception
{
private:
	string ex;
public:
	PsoptInterfaceException(string s) { ex = s; }
	const char * what () const throw ()	{ return ex.c_str(); }
};
class InputException : public PsoptInterfaceException
{
public:
	InputException(string s): PsoptInterfaceException(s) {}
};
class NoGuessException : public PsoptInterfaceException
{
public:
	NoGuessException(string s): PsoptInterfaceException(s) {}
};

struct AlgoConfigStruct
{
	string nlpMethod = "IPOPT";
	string scaling = "automatic";
	string derivatives = "automatic";
	string mesh_refinement = "automatic";
	int nlpIterMax = 500;
	double odeTolerance = 1.0e-6;
	double nlpTolerance = 1.0e-6;
	string nodes = "[10]";
};

enum FunctionType {FunNone, FunNoArgs, FunWithArgs} ;

enum GuessType {None, Linear, Circular} ;
struct GuessStruct
{
	GuessType type = None;
	union {
		struct { double begin; double end; } linear;
		struct { double begin; double end; double radiusBegin; double radiusEnd; } circular;
	};
	GuessStruct(double radiusBegin, double radiusEnd, double begin, double end)
	{
		circular.begin = begin;
		circular.end = end;
		circular.radiusBegin = radiusBegin;
		circular.radiusEnd = radiusEnd;
		type = Circular;
	}
	GuessStruct(double begin, double end)
	{
		linear.begin = begin;
		linear.end = end;
		type = Linear;
	}
	GuessStruct(double value)
	{
		linear.begin = value;
		linear.end = value;
		type = Linear;
	}
	GuessStruct()
	{
		type = None;
	}
};

struct ResultVectors {DMatrix x; DMatrix u; DMatrix t;};

/// A class providing a modern interface to PSOPT
/**
 * This class apstracts from some interlal workings of PSOPT to give the user a better
 * handling.
 */
class ModernPsoptInterface
{
private:

	std::function<void (adouble* derivatives, adouble* path, adouble* states,
			adouble* controls, adouble* parameters, adouble& time,
			adouble* xad, int iphase)>
		daeInitFunction; //!< init function for the DAE calculation
	std::function<void (adouble* e, adouble* initial_states, adouble* final_states,
			adouble* parameters,adouble& t0, adouble& tf, adouble* xad, int iphase)>
		eventInitFunction; //!< init function for the event calculation
	std::function<void (adouble* linkages, adouble* xad)> linkagesInitFunction;
	std::function<adouble (adouble* initial_states, adouble* final_states,
			adouble* parameters,adouble& t0, adouble& tf, adouble* xad, int iphase)>
	endpointCostFunction; //!< function for calculation of the endpoint cost
	std::function<adouble (adouble* states, adouble* controls, adouble* parameters,
			adouble& time, adouble* xad, int iphase)>
	integrandCostFunction; //!< function for the calculation of the cost of each phase

	struct PsoptDataStruct; //!< dummy declaration for the PSOPT-internal data structures
	unique_ptr<PsoptDataStruct> psoptData; //!< pointer to the PSOPT-internal data

	string nodesConfig; //!< configuration string for the nodes

	/// Structure containing information about a state
	/**
	 * This structure holds all the necessary information about a single state in a single phase
	 */
	struct StateStruct
	{
		double lowerBound; //!< Lower bound of the state during this stage
		double upperBound; //!< Upper bound of the state during this stage
		GuessStruct guess; //!< Guess of the values of the state
		FunctionType functionType = FunNone; //!< Type of function to calculate the derivation
		std::function<adouble()> derivationFunctionNoArgs; //!< Derivation function without argments
		std::function<adouble(adouble* states,
				adouble* controls, adouble* parameters, adouble& time,
				adouble* xad, int iphase)> derivationFunctionWithArgs; //!< Derivation function with arguments
	};
	struct statePhaseStruct
	{
		bool guessMade = false;
		vector<StateStruct> p;
	};
	struct controlStruct
	{
		double lowerBound; double upperBound;
		GuessStruct guess;
	};
	struct controlPhaseStruct
		{
			bool guessMade = false;
			vector<controlStruct> p;
		};
	struct eventStruct
	{
		double lowerBound; double upperBound;
		FunctionType functionType = FunNone;
		std::function<adouble()> eventFunctionNoArgs;
		std::function<adouble(adouble* initial_states, adouble* final_states, adouble* parameters, adouble& t0, adouble& tf, adouble* xad)> eventFunctionWithArgs;
	};
	struct pathStruct
	{
		double lowerBound; double upperBound;
		FunctionType functionType = FunNone;
		std::function<adouble()> pathFunctionNoArgs;
		std::function<adouble(adouble* states,
				adouble* controls, adouble* parameters, adouble& time,
				adouble* xad)> pathFunctionWithArgs;
	};
	struct timeStruct
	{
		double beginLower, beginUpper;
		double endLower, endUpper;
		bool guessMade = false;
		GuessStruct guess;
	};
	vector<statePhaseStruct> stateContainer;
	vector<controlPhaseStruct> controlContainer;
	vector<vector<eventStruct>> eventContainer;
	vector<vector<pathStruct>> pathContainer;
	vector<std::function<adouble()>> linkageContainer;
	vector<timeStruct> timeContainer;

	struct
	GuessMade {
		bool stat = false;
		bool control = false;
	};
	vector<GuessMade> guessMade;

	void enlargeStateContainer(int phase);
	void _AddStateDerivative(int phase, int state, std::function<adouble()> derivative);
	void _AddStateDerivative(int phase, int state, std::function<adouble(adouble* states,
			adouble* controls, adouble* parameters, adouble& time,
			adouble* xad, int iphase)> derivative);
	void _AddStateGuess(int phase, int state, GuessStruct guess);
	void _AddControlGuess(int phase, int control, GuessStruct guess);
	void enlargeControlContainer(int phase);

	void enlargeEventContainer(int phase);
	void enlargePathContainer(int phase);
	void enlargeTimeContainer(int phase);

	void verifyStateContainerSize(int phase, int state);
	void verifyControlContainerSize(int phase, int state);

	void Level1Setup(string name, string outfilename, int nlinkages);
	void Level2Setup();
	void Level2SetupPhase(int phase);
	void Level3Setup();
	void AlgoSetup(AlgoConfigStruct algoConfig);
	/// Setup the relevant PSOPT Functions
	/**
	 *
	 */
	void FunctionSetup();
	void prePsoptFunction();

	DMatrix GuessToDMatrix(GuessStruct guess, int nodes);
	void dae(adouble* derivatives, adouble* path, adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase);
public:
	bool display = true,    //!< \anchor display [in] Show debugging output during problem creation
	displayTimes = true,    //!< [in] Show Times (requires \ref display to be true)
	displayStates = true,   //!< [in] Show States (requires \ref display to be true)
	displayControls = true, //!< [in] Show Controls (requires \ref display to be true)
	displayEvents = true,   //!< [in] Show Events (requires \ref display to be true)
	displayPaths = true;    //!< [in] Show Paths (requires \ref display to be true)

	/// Constructor for the Interface.
	ModernPsoptInterface();

	/// Destructor for the Interface.
	/**
	 * 	Automatic deallocation of unique_ptr<PsoptDataStruct> psoptData.
	 */
	~ModernPsoptInterface();

	/// Set the init function for DAE.
	/**
	 * This is always called before the functions specified in CreateNewState,
	 * AddStateDerivative and CreateNewPath.
	 * @param daeFunction [in] a function to run at the beginning of a DAE calculation.
	 * For a description of the arguments to daeFunction, refer to the PSOPT documentation.
	 * This function should set the contents of derivatives and path according to the other values.
	 */
	void SetDaeInitFunction(std::function<void (
			adouble* derivatives, //!< [out] derivatives the derivative of the states
			adouble* path, adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase)> daeFunction);
	void SetEventInitFunction(std::function<void (adouble* e, adouble* initial_states, adouble* final_states, adouble* parameters, adouble& t0, adouble& tf, adouble* xad, int iphase)> eventFunction);
	void SetLinkagesInitFunction(std::function<void (adouble* linkages, adouble* xad)> linkageFunction);
	void SetEndpointCostFunction(std::function<adouble (adouble* initial_states, adouble* final_states, adouble* parameters,adouble& t0, adouble& tf, adouble* xad, int iphase)> ecFunction);
	void SetIntegrandCostFunction(std::function<adouble (adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase)> icFunction);
	int CreateNewState(int phase, double lower, double upper);
	int CreateNewState(int phase, double lower, double upper, GuessStruct guess);
	int CreateNewState(int phase, double lower, double upper, GuessStruct guess, std::function<adouble()> derivative);
	int CreateNewState(int phase, double lower, double upper, std::function<adouble(adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase)> derivative);
	int CreateNewState(int phase, double lower, double upper, GuessStruct guess, std::function<adouble(adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase)> derivative);
	void AddStateDerivative(int phase, int state, std::function<adouble(adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad, int iphase)> derivative);
	void AddStateGuess(int phase, int state, GuessStruct guess);
	int CreateNewControl(int phase, double lower, double upper);
	int CreateNewControl(int phase, double lower, double upper, GuessStruct guess);
	void AddControlGuess(int phase, int control, GuessStruct guess);
	int CreateNewEvent(int phase, double lower, double upper);
	int CreateNewEvent(int phase, double lower, double upper, std::function<adouble(adouble* initial_states, adouble* final_states, adouble* parameters, adouble& t0, adouble& tf, adouble* xad)> eventFunction);
	int CreateNewPath(int phase, double lower, double upper);
	int CreateNewPath(int phase, double lower, double upper, std::function<adouble(adouble* states, adouble* controls, adouble* parameters, adouble& time, adouble* xad)> pathFunction);

	void SetTimeConstraints(int phase, double beginLower, double beginUpper, double endLower, double EndUpper);
	void SetTimeConstraints(int phase, double beginLower, double beginUpper, double endLower, double EndUpper, GuessStruct guess);
	void Psopt(string name, string outfilename, int nlinkages, AlgoConfigStruct algoConfig);
	void plot_win(DMatrix& theta, DMatrix& r, string title, string xlabel, string ylabel, string legend, string terminal, string output, bool show = true);
	ResultVectors getResults();
	vector<adouble> getEndTimesOfPhases();

};


} // end namespace
