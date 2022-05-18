function output = copyprops(input, output)
%Copy dot-indexables from input to output, which can be either an object or
%struct.
%
%     output = copyprops(input, output)
%     output = copyprops(input)
%
%The input arguments "input" and "output" can be arrays of the same size,
%or "input" can be a scalar.
%
%The second syntax will return a struct with only the copied property
%values. When "input" is an object, only non-Dependent properties are
%copied. When "input" is a struct, fieldnames will be copied.

   if isstruct(input)
    
     pNames=fieldnames(input);
       
   else

     C = metaclass(input);
     P = C.Properties;

     pNames=cellfun(@(c) c.Name,P,'uni',0);
     nonDep=cellfun(@(c) ~c.Dependent,P);
     pNames=pNames(nonDep);
     
   end
   
   if nargin<2

       
       
       for k = 1:length(pNames)     
           output(1).(pNames{k}) = input(1).(pNames{k});
       end
       
     output(1:numel(input))=output(1);
     output=reshape(output,size(input)); 
     
     output=copyprops(input,output);
     return
   end
   
    Ni=numel(input);
    No=numel(output);

    if Ni>1 && ( No~=Ni || ~isequal( size(input),size(output) ) )
       error 'Incompatible sizes' 
    elseif Ni==1
        
       input(1:numel(output))=input(1);
       input=reshape(input,size(output)); 
     
    end
    
   for k = 1:length(pNames)
      for i=1:No
        output(i).(pNames{k}) = input(i).(pNames{k});
      end
   end