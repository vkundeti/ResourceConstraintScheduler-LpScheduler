#ifndef STRIP_PACKING_UNIT_TEST_UTILS
#define STRIP_PACKING_UNIT_TEST_UTILS

#include <scheduler/feasible_scheduler.hpp>
#include <scheduler/scheduler_unit_test_utils.hpp>
#include "2d_strip_packing_lp.hpp"

namespace mv {
namespace strip_packing_unit_tests {

class Operation_Dag : public scheduler_unit_tests::Operation_Dag {
  public:
    using scheduler_unit_tests::Operation_Dag::Operation_Dag;
    typedef Operation_Dag dag_t;
    typedef int unit_t;

    static unit_t width(const dag_t& dag, const operation_t& op) {
      return (unit_t) dag.get_operation_resources(op);
    }

    static unit_t height(const dag_t& dag, const operation_t& op) {
      return (unit_t) dag.get_operation_delay(op);
    }

}; // class Operation_Dag //


// Using the Cumulative_Resource_State //
typedef mv::lp_scheduler::Cumulative_Resource_State<
  Operation_Dag::resource_t, Operation_Dag::operation_t> resource_state_t;

} // namespace strip_packing_unit_tests //
} // namespace mv //




namespace mv {
namespace lp_scheduler {

template<>
struct scheduler_traits<strip_packing_unit_tests::Operation_Dag>
  : public strip_packing_unit_tests::Operation_Dag {
    ////////////////////////////////////////////////////////////////////////////
    // input graph model and delay model are used from Operation_Dag itself   //
    using strip_packing_unit_tests::Operation_Dag::Operation_Dag;

    ////////////////////////////////////////////////////////////////////////////
    // define resource update model //
    typedef strip_packing_unit_tests::resource_state_t resource_state_t;

    static void initialize_resource_upper_bound(const resource_t& upper_bound,
        resource_state_t& state) {
      state.initialize_resource_upper_bound(upper_bound);
    }

    static bool is_empty_demand(const resource_t& demand) {
      return (demand == resource_t(0UL));
    }

    static bool is_resource_available(const resource_t& demand,
          const resource_state_t& state) {
      return state.is_resource_available(demand);
    }

    static bool schedule_operation(const operation_t& op,
        const resource_t& demand, resource_state_t& state,
        const_operation_iterator_t, const_operation_iterator_t) {
      return state.assign_resources(op, demand);
    }

    static bool unschedule_operation(const operation_t& op,
        resource_state_t& state) {
      return state.unassign_resources(op);
    }

    template<typename T>
    static size_t scheduled_op_time(const T& in) { return in.time_; }

    template<typename T>
    static operation_t scheduled_op(const T& in) { return in.op_; }


    ////////////////////////////////////////////////////////////////////////////
    // Scheduled Operation Traits //
    struct scheduled_op_t {
      scheduled_op_t() : op_(), schedule_time_(), rbegin_(), rend_() {}
      scheduled_op_t(const operation_t& op, size_t schedule_time,
          resource_t rbegin, resource_t rend) : op_(op),
      schedule_time_(schedule_time), rbegin_(rbegin), rend_(rend) {}

      bool operator<(const scheduled_op_t& o) const {
        return schedule_time_ < o.schedule_time_;
      }

      // interval equivalence //
      bool operator==(const scheduled_op_t& o) const {
        return (rbegin_ == o.rbegin_) && (rend_ == o.rend_);
      }

      operation_t op_;
      size_t schedule_time_;
      resource_t rbegin_;
      resource_t rend_;
    }; // struct scheduled_op_t //
    typedef std::hash<operation_t> scheduled_op_hash_t;
    typedef size_t unit_t;

    struct data_op_selector_t {
      data_op_selector_t() : dag_ptr_(NULL) {}
      data_op_selector_t(const dag_t& dag) : dag_ptr_(&dag) {}
      data_op_selector_t(const data_op_selector_t& o) : dag_ptr_(o.dag_ptr_) {}

      bool operator()(const operation_t& op) const {
        assert(dag_ptr_);
        return dag_ptr_->is_data_op(op);
      }
      dag_t const * dag_ptr_;
    }; // struct data_op_selector_t //
    typedef size_t schedule_time_t;

    static resource_t begin_resource(const scheduled_op_t& op) {
      return op.rbegin_;
    }
    static resource_t end_resource(const scheduled_op_t& op) {
      return op.rend_;
    }

    static operation_t scheduled_operation(const scheduled_op_t& op) {
      return op.op_;
    }

    static schedule_time_t scheduled_time(const scheduled_op_t& op) {
      return op.schedule_time_;
    }
    static bool using_valid_resource(const scheduled_op_t& op) {
      return (op.rbegin_ <= op.rend_);
    }
    static bool is_valid_scheduled_op(const scheduled_op_t& ) { return true; }
    static void set_new_schedule_time(scheduled_op_t& op,
          const schedule_time_t& t) {
      op.schedule_time_ = t;
    }


    // DAG editor traits //
    struct dag_editor_t {
      //////////////////////////////////////////////////////////////////////////
      struct edge_t {
        edge_t(operation_t src, operation_t snk) : src_(src), sink_(snk) {}
        operation_t src_;
        operation_t sink_;
        void print() const {
          std::cout << " ( " << src_ << ", " << sink_ << ")" << std::endl;
        }
      }; // struct edge_t // 
      typedef std::list<edge_t> edge_list_t;
      typedef std::list<operation_t> node_list_t;
      typedef std::list<std::pair<operation_t,
              std::pair<operation_t, resource_t> > > node_pair_list_t;
      //////////////////////////////////////////////////////////////////////////

      dag_editor_t(const dag_t& dag) : dag_(dag), added_edges_(), removed_edges_(),
        added_nodes_() {}


      void print() const {
        printf("[ADDED EDGES]:\n");
        for (edge_t edge : added_edges_) {
          edge.print();
        }

        printf("[REMOVED EDGES]:\n");
        for (edge_t edge : removed_edges_) {
          edge.print();
        }

        printf("[ADDED NODES]:\n");
        for (auto node : added_nodes_) {
          std::cout << node.first << std::endl;
        }
      }

      void apply_edits(dag_t& input_dag) const {
        // get the adjacency map //
        for (edge_t e : removed_edges_) {
          input_dag.remove_edge(e.src_, e.sink_);
        }

        for (auto node : added_nodes_) {
          input_dag.add_node(node.first);
          // update delay and resource utility of this op //
          input_dag.set_operation_delay(node.first,
              input_dag.get_operation_delay(node.second.first));
          input_dag.set_operation_resources(node.first, (node.second.second) );
        }

        for (edge_t e : added_edges_) {
          input_dag.add_edge(e.src_, e.sink_);
        }
      }

      const dag_t &dag_;
      edge_list_t added_edges_;
      edge_list_t removed_edges_;
      node_pair_list_t added_nodes_;
    }; // struct dag_editor_t //

    static operation_t new_operation(dag_editor_t& dag_editor,
        const std::string& name, operation_t orig_op, resource_t r) {
      dag_editor.added_nodes_.push_back(
          std::make_pair(operation_t(name), std::make_pair(orig_op,r) ) );
      return dag_editor.added_nodes_.back().first;
    }

    static void add_edge(dag_editor_t& dag_editor, const operation_t& src, 
        const operation_t& sink) {
      dag_editor.added_edges_.push_back(dag_editor_t::edge_t(src, sink));
    }

    static void remove_edge(dag_editor_t& dag_editor, const operation_t& src, 
        const operation_t& sink) {
      dag_editor.removed_edges_.push_back(dag_editor_t::edge_t(src, sink));
    }

    static const dag_t& get_dag(const dag_editor_t& editor) {
      return editor.dag_;
    }

    static void apply_edits(const dag_editor_t& editor, dag_t& input_dag ) {
      editor.apply_edits(input_dag);
    }


}; // specialization for strip_packing_unit_tests::dag_t //


} // namespace lp_scheduler //
} // namespace mv //


typedef mv::lp_scheduler::scheduler_traits<
  mv::strip_packing_unit_tests::Operation_Dag > strip_packing_unit_testing_t;
typedef typename strip_packing_unit_testing_t::scheduled_op_t
  strip_packing_unit_testing_scheduled_op_t;

namespace mv {
namespace lp_scheduler {

template<>
struct interval_traits<strip_packing_unit_testing_scheduled_op_t> {
    typedef strip_packing_unit_testing_scheduled_op_t interval_t;
    typedef size_t unit_t;
    // interval traits //
    static unit_t interval_begin(const interval_t& op) { return op.rbegin_; }
    static unit_t interval_end(const interval_t& op) { return op.rend_; }
}; // struct interval_traits //


} // namespace lp_scheduler //
} // namespace mv//

typedef typename mv::lp_scheduler::Feasible_Memory_Schedule_Generator<
	mv::strip_packing_unit_tests::Operation_Dag > sp_scheduler_t;
#endif
