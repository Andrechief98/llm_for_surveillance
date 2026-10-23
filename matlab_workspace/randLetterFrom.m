function [letter] = randLetterFrom(letters)
% It returns a random letter from a given array of letters.
% Example:
%   letter = randLetterFrom(["A","B","C"]) 

    if isempty(letters)
        error('Empty array of letters.');
    end

    idx = randi(numel(letters));
    letter = letters(idx);
end

