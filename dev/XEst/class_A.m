classdef class_A < matlab.System
  % untitled2 Add summary here
  %
  % This template includes the minimum set of functions required
  % to define a System object with discrete state.

  % Public, tunable properties
  properties
      title = 'hello'
      book = 'physics'
      level = 'first'
  end

  properties(DiscreteState)

  end

  % Pre-computed constants
  properties(Access = private)

  end

  methods(Access = protected)
    function setupImpl(obj)
      % Perform one-time calculations, such as computing constants
    end

    function y = stepImpl(obj,u)
      % Implement algorithm. Calculate y as a function of input u and
      % discrete states.
      y = u;
    end

    function resetImpl(obj)
      % Initialize / reset discrete-state properties
    end
  end
end
