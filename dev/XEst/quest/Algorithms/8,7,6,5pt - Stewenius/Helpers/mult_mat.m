function C = mult_mat(A,B)

C = cell(3,3);

if isa(A, 'double')
  for row=1:3
    for col=1:3
       C{row,col} = mult_atom(A, B{row,col});
    end
  end
else
  for row=1:3
    for col=1:3
       temp = zeros(1,30);
       for i=1:3
          temp = temp + mult_atom(A{row,i},B{i,col});
          C{row,col} = temp;
       end
    end
  end
end
