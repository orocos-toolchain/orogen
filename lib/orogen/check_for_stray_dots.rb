module OroGen
            # Check for the case when there is an superfluous dot at
            # the end of a task statement
            def self.check_for_stray_dots(filename, name, args)
                # Building the regular expression to 
                # match on the method name and arguments
                regexp_expression = "#{name}.*"
                args.each do |element|
                    regexp_expression << "#{element}.*"
                end
                regexp = Regexp.new(regexp_expression)

                # Check the spec to locate the error in case
                # of stray dots
                File.open(filename) do |file|
                    begin 
                        line_counter = 0
                        previous_non_empty_line_number = 0
                        previous_non_empty_line = nil
                        while true
                            line = file.readline
                            line_counter += 1
                            if regexp.match(line)
                                if previous_non_empty_line =~ /\.$/
                                    raise ArgumentError, "stray dot in statement: #{previous_non_empty_line.strip} (line #{previous_non_empty_line_number})"
                                end
                            end

                            if line =~ /.+/
                                previous_non_empty_line = line
                                previous_non_empty_line_number = line_counter
                            end
                        end
                    rescue EOFError
                    end
                end
            end
end
