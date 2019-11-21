# frozen_string_literal: true

module OroGen
    def self.warn_deprecated(method_name, msg)
        OroGen.warn "#{method_name} is deprecated: #{msg}"
    end
end
