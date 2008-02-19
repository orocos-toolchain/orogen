std::string <%= toolkit_name %>ToolkitPlugin::getName() { return "<%= toolkit_name %>"; }
bool <%= toolkit_name %>ToolkitPlugin::loadTypes()
{
    TypeInfoRepository::shared_ptr ti = TypeInfoRepository::Instance();
    <% generated_types.each do |type| %>ti->addType( new <%= type.basename %>TypeInfo() );<% end %>
    return true;
}

<%= toolkit_name %> ToolkitPlugin <%= toolkit_name >Toolkit;
