#ifndef VIEW_EVALUATOR_IG_H
#define VIEW_EVALUATOR_IG_H

#include <victim_localization/view_generator_ig.h>
#include <victim_localization/victim_map_base.h>


class view_evaluator_IG
{
public:
  float info_selected_utility_;

  view_evaluator_IG();
  Pose getTargetPose();
  void setViewGenerator(view_generator_IG* v);
  void setMappingModule(Victim_Map_Base* m);
  void update_parameters();
  void evaluate();


protected:
  view_generator_IG *view_gen_;
  Victim_Map_Base *mapping_module_;
  Pose current_pose_;
  double current_yaw_;
  std::string MapLayer;
  Pose selected_pose_;

  double HFOV_deg;
  double max_depth_d;
  double x_arena_max;
  double y_arena_max;
  double const_;
  double calculateIG(Pose p);
  double getCellEntropy(Index cell_);

};

#endif // VIEW_EVALUATOR_IG_H
