function normedv = normalize_all(vectors)
  normedv  = zeros(size(vectors));
  for v = 1: size(vectors, 2)
    normedv(:, v) = normalizeVec(vectors(:, v));
  end
