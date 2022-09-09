#ifndef SCHEDULER_UNIT_TEST_UTILS_HPP
#define SCHEDULER_UNIT_TEST_UTILS_HPP

#include <exception>
#include <iostream>

#include "scheduler/feasible_scheduler.hpp"

namespace mv_unit_tests {

struct interval_t {
  interval_t(int b, int e, const std::string& id="")
    : beg_(b), end_(e), id_(id) {}

  interval_t() : beg_(), end_(), id_() {}

  bool operator==(const interval_t& o) const {
    return (beg_ == o.beg_) && (end_ == o.end_) && (id_ == o.id_);
  }

  void print() const {
    std::cout << "[ " << beg_ << " " << end_ << " " << id_ << "]" << std::endl;
  }

  int beg_;
  int end_;
  std::string id_;
}; // struct interval_t //

} // namespace mv_unit_tests //

namespace mv {
namespace lp_scheduler {

template<>
struct interval_traits<mv_unit_tests::interval_t> {
  typedef int unit_t;
  typedef mv_unit_tests::interval_t interval_t;



  static unit_t interval_begin(const interval_t& interval) {
    return interval.beg_;
  }

  static unit_t interval_end(const interval_t& interval) {
    return interval.end_;
  }

  static void set_interval(interval_t& interval,
        const unit_t& beg, const unit_t& end) {
    interval.beg_ = beg; 
    interval.end_ = end;
  }

}; // struct interval_traits //

} // namespace lp_scheduler //
} // namespace mv //


namespace scheduler_unit_tests {


// Simple DAG for unit testing the core algorithm //
class Operation_Dag {
  public:
    typedef Operation_Dag dag_t;
    typedef std::string operation_t;
    typedef std::hash<std::string> operation_hash_t;
    typedef std::vector<operation_t> adjacency_list_t;
    typedef typename adjacency_list_t::const_iterator const_adj_list_iterator_t;
    typedef std::unordered_map<operation_t, adjacency_list_t> adjacency_map_t;
    typedef typename adjacency_map_t::const_iterator const_adj_map_iterator_t;

    // resource cost model //
    typedef size_t resource_t;
    typedef std::unordered_map<operation_t, resource_t> resource_cost_model_t;
    typedef typename resource_cost_model_t::iterator resource_cost_iterator_t;
    typedef std::unordered_set<operation_t> data_op_set_t;
    typedef std::unordered_map<operation_t, operation_t> inplace_op_map_t;

    // delay cost model //
    typedef size_t delay_t;
    typedef std::unordered_map<operation_t, delay_t> delay_cost_model_t;

    class const_operation_iterator_t {
      public:
        const_operation_iterator_t() : adj_map_ptr_(NULL),
          itr_begin_(), itr_end_(), itr_adj_begin_(), itr_adj_end_(),
          iterate_edges_(false) {}

        const_operation_iterator_t(const const_operation_iterator_t& o)
          : adj_map_ptr_(), itr_begin_(), itr_end_(), itr_adj_begin_(),
            itr_adj_end_(), iterate_edges_(false) {
            adj_map_ptr_ = o.adj_map_ptr_;
            itr_begin_ = o.itr_begin_;
            itr_end_ = o.itr_end_;
            itr_adj_begin_ = o.itr_adj_begin_;
            itr_adj_end_ = o.itr_adj_end_;
            iterate_edges_ = o.iterate_edges_;
         }

        const_operation_iterator_t(const adjacency_map_t& adj_map,
            const_adj_map_iterator_t begin, const_adj_map_iterator_t end,
            bool iterate_edges=false) : adj_map_ptr_(&adj_map), itr_begin_(begin),
          itr_end_(end), itr_adj_begin_(), itr_adj_end_(),
          iterate_edges_(iterate_edges)
        {

          if (iterate_edges_ && (itr_begin_ != itr_end_)) {
            itr_adj_begin_ = (itr_begin_->second).cbegin();
            itr_adj_end_ = (itr_begin_->second).cend();
            if (itr_adj_begin_ == itr_adj_end_) {
              move_to_next_edge();
            }
          }
        }

        // only invalid iterators are equivalent //
        bool operator==(const const_operation_iterator_t& o) const {
          return !is_valid() && !o.is_valid();
        }

        bool operator!=(const const_operation_iterator_t& o) const {
          return !(*this == o);
        }

        const const_operation_iterator_t& operator++() {
          move_to_next_vertex_or_edge();
          return *this;
        }

        const const_operation_iterator_t& operator=(
            const const_operation_iterator_t& o) {
          if (this != &o) {
            adj_map_ptr_ = o.adj_map_ptr_;
            itr_begin_ = o.itr_begin_;
            itr_end_ = o.itr_end_;
            itr_adj_begin_ = o.itr_adj_begin_;
            itr_adj_end_ = o.itr_adj_end_;
            iterate_edges_ = o.iterate_edges_;
          }
          return *this;
        }

        const operation_t& operator*() const {
          const_adj_map_iterator_t op_itr = itr_begin_;
          if (iterate_edges_) {
            op_itr = adj_map_ptr_->find(*itr_adj_begin_);
            assert(op_itr != adj_map_ptr_->end());
          }
          return op_itr->first;
        }


      private:

        // Precondition: is_valid() //
        bool move_to_next_vertex_or_edge() {
          return iterate_edges_ ? move_to_next_edge() :
              (++itr_begin_) == itr_end_;
        }

        bool move_to_next_edge() {

          if (itr_adj_begin_ != itr_adj_end_) {
            // move to edge if possible //
            ++itr_adj_begin_;
          }

          if (itr_adj_begin_ == itr_adj_end_) {
            ++itr_begin_; 
            // find a node with atleast one out going edge //
            while ( (itr_begin_ != itr_end_) &&
                    ( (itr_adj_begin_ = (itr_begin_->second).begin()) ==
                      (itr_adj_end_ = (itr_begin_->second).end()) ) ) {
              ++itr_begin_;
            }
          }

          return (itr_adj_begin_ != itr_adj_end_);
        }

        bool is_valid(void) const { return itr_begin_ != itr_end_; }

        adjacency_map_t const * adj_map_ptr_;
        const_adj_map_iterator_t itr_begin_;
        const_adj_map_iterator_t itr_end_;
        const_adj_list_iterator_t itr_adj_begin_;
        const_adj_list_iterator_t itr_adj_end_;
        bool iterate_edges_;
    }; // class const_operation_iterator_t //

    class operation_dag_exception_t : public std::exception {
      public:
        operation_dag_exception_t(
            const char *exception_str="operation_dag_exception_t:")
          : str_(exception_str) {}

        virtual const char* what() const throw () {
          return str_.c_str();
        }

      private:
        const std::string str_;
    }; // class operation_dag_exception_t //


    Operation_Dag(const adjacency_map_t& in) : adj_map_(in) {init();} 
    Operation_Dag() : adj_map_() {} 

    void print() const {
      for (auto itr=adj_map_.begin(); itr!=adj_map_.end(); ++itr) {
        printf("[%s]->{ ", (itr->first).c_str());
        for (auto op : itr->second) {
          printf("%s, ", op.c_str());
        }
        printf("}\n");
      }
    }

    const_operation_iterator_t begin(const operation_t& op) const {
      adjacency_map_t::const_iterator itr = adj_map_.find(op), itr_next;
      if (itr == adj_map_.end()) { return const_operation_iterator_t(); }

      itr_next = itr;
      ++itr_next;
      return const_operation_iterator_t(adj_map_, itr, itr_next, true);
    }

    const_operation_iterator_t begin_in(const operation_t& op) const {
      adjacency_map_t::const_iterator itr = inv_adj_map_.find(op), itr_next;
      if (itr == inv_adj_map_.end()) { return const_operation_iterator_t(); }

      itr_next = itr;
      ++itr_next;
      return const_operation_iterator_t(adj_map_, itr, itr_next, true);
    }

    const_operation_iterator_t end(const operation_t& op) const {
      (void) op;
      return const_operation_iterator_t();
    }

    const_operation_iterator_t begin() const { 
      return const_operation_iterator_t(adj_map_, adj_map_.begin(),
          adj_map_.end(), false);
    }

    const_operation_iterator_t begin_edges() const {
      return const_operation_iterator_t(adj_map_, adj_map_.begin(),
          adj_map_.end(), true);
    }

    const_operation_iterator_t end() const {
      return const_operation_iterator_t();
    }

    void reset(const adjacency_map_t& adj_map) {
      adj_map_ = adj_map;
      init();
    }

    template<typename NodeIterator>
    void remove_nodes(NodeIterator nbegin, NodeIterator nend) {
      for (;nbegin!=nend; ++nbegin) { remove_node(*nbegin); }
      init();
    }

    void remove_node(operation_t u) {
      // STEP-1: a. remove incoming edges { (p, u) }
      //         b. remove outgoing edges { (u,c) }
      // STEP-2: remove the node u all the following maps:
      //         -1. adj_map_
      //          0. inv_adj_map_
      //          1. delay_cost_model_
      //          2. resource_cost_model_
      //          3. data_op_set_
      //          4. inplace_op_map_
      // STEP-3: init() 

      {
        // REMOVE all incoming edges //
        adjacency_map_t::iterator pitr = inv_adj_map_.find(u);
        if (pitr == inv_adj_map_.end()) {
          throw operation_dag_exception_t("remove_node: "
                "node missing in the DAG");
        }

        // STEP-1.a : adj_inv_map_ constant while modifyin adj_map_ //
        const adjacency_list_t &adj_list = pitr->second;
        for (adjacency_list_t::const_iterator litr=adj_list.begin();
              litr!=adj_list.end();++litr) {
          // incoming edge: (*litr, u) // 
          remove_from_adj_list(adj_map_, *litr, u);
        }
        inv_adj_map_.erase(pitr);
      }

      {
        // REMOVE all outgoing edges //
        adjacency_map_t::iterator citr = adj_map_.find(u);
        if (citr == adj_map_.end()) {
          throw operation_dag_exception_t("remove_node: "
                "node missing in the DAG");
        }

        // STEP-1.a : adj_inv_map_ constant while modifyin adj_map_ //
        const adjacency_list_t &adj_list = citr->second;
        for (adjacency_list_t::const_iterator litr=adj_list.begin();
              litr!=adj_list.end();++litr) {
          // outgoing edge: (u, *litr) // 
          remove_from_adj_list(inv_adj_map_, *litr, u);
        }
        adj_map_.erase(citr);
      }
      // NOTE: look at the beautiful symmetry between both the above blocks //

      //remove_node_from_table(adj_map_, u); // already erased 
      //remove_node_from_table(inv_adj_map_, u); // already erased 
      remove_node_from_table(delay_cost_model_, u);
      remove_node_from_table(resource_cost_model_, u);
      remove_node_from_table(data_op_set_, u);
      remove_node_from_table(inplace_op_map_, u);
    }

    // Node gets inserted if and returns iterator of from the adjacency map //
    adjacency_map_t::iterator add_node(operation_t u) {
     auto itr = adj_map_.find(u);
     if (itr != adj_map_.end()) {
       throw operation_dag_exception_t("add_node: node already exists");
     }

     auto inv_itr = inv_adj_map_.find(u);
     if (inv_itr != inv_adj_map_.end()) {
       throw operation_dag_exception_t("add_node: node already exists");
     }

     inv_adj_map_.insert(std::make_pair(u, adjacency_list_t()));
     return (adj_map_.insert(std::make_pair(u, adjacency_list_t() ))).first;
    }

    void remove_edge(operation_t u, operation_t v) {
      // STEP-1: remove v from adj_map_ of u //
      remove_from_adj_list(adj_map_, u, v);
      // STEP-2: remove u from inv_adj_map_ of v //
      remove_from_adj_list(inv_adj_map_, v, u);
    }

    // NOTE: if u or v does not exist then it will be added to the DAG.
    void add_edge(operation_t u, operation_t v) {
      adjacency_map_t::iterator uitr = adj_map_.find(u);
      adjacency_map_t::iterator vitr = adj_map_.find(v);

      if (uitr == adj_map_.end()) { add_node(u); }
      if (vitr == adj_map_.end()) { add_node(v); }

      // STEP-1: add v to adj_map_ of u //
      add_to_adj_list(adj_map_, u, v);
      // STEP-2: add u from inv_adj_map_ of v //
      add_to_adj_list(inv_adj_map_, v, u);
    }

    // Precondition: new delay model most have an entry for every op. //
    void reset_delay_model(const delay_cost_model_t& delay_model) {
      delay_cost_model_.clear();
      delay_cost_model_ = delay_model;
    }

    // Precondition: new delay model most have an entry for every op. //
    void reset_resource_model(const resource_cost_model_t& resource_model) {
      resource_cost_model_.clear();
      resource_cost_model_ = resource_model;
    }

    void reset_resource_utility(const operation_t& op, resource_t new_utility) {
      resource_cost_iterator_t itr = resource_cost_model_.find(op);
      assert(itr != resource_cost_model_.end());
      itr->second = new_utility;
    }

    const operation_t& get_operation(const std::string& name) const {
      adjacency_map_t::const_iterator itr = adj_map_.find(name);
      assert(itr != adj_map_.end());
      return itr->first;
    }


    delay_t get_operation_delay(const operation_t& op) const {
      resource_cost_model_t::const_iterator itr = delay_cost_model_.find(op);
      assert(itr != delay_cost_model_.end());
      return itr->second;
    }

    resource_t get_operation_resources(const operation_t& op) const {
      resource_cost_model_t::const_iterator itr = resource_cost_model_.find(op);
      assert(itr != resource_cost_model_.end());
      return itr->second;
    }

    void set_operation_delay(const operation_t& op, delay_t d) {
      resource_cost_model_t::iterator itr = delay_cost_model_.find(op);
      if (itr == delay_cost_model_.end()) {
        delay_cost_model_.insert(std::make_pair(op, d));
      } else {
        itr->second = d;
      }
    }

    void set_operation_resources(const operation_t& op, resource_t r) {
      resource_cost_model_t::iterator itr = resource_cost_model_.find(op);
      if (itr == resource_cost_model_.end()) {
        resource_cost_model_.insert(std::make_pair(op, r));
      } else {
        itr->second = r;
      }
    }

    bool is_inplace_op(const operation_t& op) const {
      return inplace_op_map_.find(op) != inplace_op_map_.end();
    }

    //Precondition: is_inplace_op(op) is true //
    operation_t get_inplace_output_op(const operation_t& op) const {
      auto itr = inplace_op_map_.find(op);
      return itr->second;
    }

    size_t size() const { return adj_map_.size(); }

    bool is_data_op(const operation_t& op) const {
      return !(data_op_set_.find(op) == data_op_set_.end());
    }

    void reset_data_op_set(const data_op_set_t& in) { data_op_set_ = in; }
    void reset_inplace_op_map(const inplace_op_map_t& in) {
      inplace_op_map_ = in;
    }

    ////////////////////////////////////////////////////////////////////////////
    // scheduler_traits //

    static const char * operation_name(const operation_t& op) {
      return op.c_str();
    }
    static const_operation_iterator_t operations_begin(const dag_t& g) {
      return g.begin();
    }

    static const_operation_iterator_t operations_end(const dag_t& g) {
      return g.end();
    }

    static const_operation_iterator_t outgoing_operations_begin(const dag_t& g,
        const operation_t& op) { return g.begin(op); }

    static const_operation_iterator_t outgoing_operations_end(const dag_t& g,
        const operation_t& op) { return g.end(op); }

    static const_operation_iterator_t incoming_operations_begin(const dag_t& g,
        const operation_t& op) { return g.begin_in(op); }

    static bool is_pseudo_input_edge(const dag_t& , const operation_t& ,
        const operation_t& sink) { return false; }

    static bool is_inplace_op(const dag_t& dag, const operation_t& op) {
      return dag.is_inplace_op(op);
    }
    static operation_t get_inplace_output_op(
        const dag_t& dag, const operation_t& op) {
      return dag.get_inplace_output_op(op);
    }



    static const_operation_iterator_t incoming_operations_end(const dag_t& g,
        const operation_t& op) { return g.end(op); }

    static delay_t delay(const dag_t& dag, const operation_t& op) {
      return dag.get_operation_delay(op);
    }

    // TODO(vamsikku) : change this to specific values of spilled delay //
    static delay_t spilled_read_delay(const dag_t& , const operation_t& ) {
      return delay_t(1);
    }

    static delay_t spilled_write_delay(const dag_t& , const operation_t& ){
      return delay_t(1);
    }

    static resource_t resource_utility(const dag_t& dag, const operation_t& op)
    {
      return dag.get_operation_resources(op);
    }

    static bool is_data_operation(const dag_t& dag, const operation_t& op) {
      return dag.is_data_op(op);
    }

    static bool is_compute_operation(const dag_t& dag, const operation_t& op) {
      return !dag.is_data_op(op);
    }

    ////////////////////////////////////////////////////////////////////////////

    bool is_valid_edge(operation_t u, operation_t v) const {
      {
        auto itr = adj_map_.find(u);
        if (itr == adj_map_.end()) { return false; }

        bool u_v_exists = false;
        for (operation_t op : itr->second) {
          if (op == v) {
            u_v_exists = true;
            break;
          }
        }
        if (!u_v_exists) { return false; }
      }


      {
        auto itr = inv_adj_map_.find(v);
        if (itr == inv_adj_map_.end()) { return false; }

        bool v_u_exists = false;
        for (operation_t op : itr->second) {
          if (op == u) {
            v_u_exists = true;
            break;
          }
        }
        return v_u_exists;
      }
    }

  private:

    template<typename T>
    void remove_node_from_table(T& map, operation_t op) {
      auto itr = map.find(op);
      if (itr == map.end()) {
        throw operation_dag_exception_t("remove_node: node missing in the DAG");
      }
      map.erase(itr);
    }

    // removes v from adj_list of u //
    void remove_from_adj_list(adjacency_map_t& amap,
          operation_t u, operation_t v) {

      typename adjacency_map_t::iterator itr = amap.find(u);
      if (itr == amap.end()) {
        throw operation_dag_exception_t("remove_from_adj_list: "
              "key missing in adjacency map");
      }

      adjacency_list_t &adj_list = itr->second;
      typename adjacency_list_t::iterator litr = adj_list.begin();

      while ( (litr != adj_list.end()) && !(*litr == v)) { ++litr; }

      if (litr == adj_list.end()) {
        throw operation_dag_exception_t("remove_edge[adj_map]: missing edge ");
      }

      adj_list.erase(litr);
    }

    void add_to_adj_list(adjacency_map_t& amap, operation_t u, operation_t v) {
      typename adjacency_map_t::iterator itr = amap.find(u);

      if (itr == amap.end()) {
        throw operation_dag_exception_t("remove_from_adj_list: "
              "key missing in adjacency map");
      }

      adjacency_list_t &adj_list = itr->second;
      typename adjacency_list_t::iterator litr = adj_list.begin();

      while ( (litr != adj_list.end()) && !(*litr == v)) { ++litr; }

      if (litr != adj_list.end()) {
        throw operation_dag_exception_t("add_edge[adj_map]: edge_exists");
      }

      adj_list.push_back(v);
    }




    void init() {
      reset_to_uniform_delay_model();
      reset_to_uniform_resource_model();
      init_inv_adj_map();
    }

    // Precondition: adj_map_ must be constructed //
    void init_inv_adj_map() {
      inv_adj_map_.clear();
      const_adj_map_iterator_t itr = adj_map_.begin(), itr_end = adj_map_.end();

      for (; itr != itr_end; ++itr) {
        const_adj_list_iterator_t inv_itr = (itr->second).begin(),
                                  inv_itr_end = (itr->second).end();
        for (; inv_itr != inv_itr_end; ++inv_itr) {
          inv_adj_map_[ *inv_itr ].push_back( itr->first);
        }
      }
    }

    void reset_to_uniform_delay_model(delay_t d=1) {
      delay_cost_model_.clear();

      const_operation_iterator_t itr=begin(), itr_end=end();

      while (itr != itr_end) {
        delay_cost_model_[*itr] = delay_t(d);
        ++itr;
      }
    }

    void reset_to_uniform_resource_model(resource_t r=1) {
      resource_cost_model_.clear();

      const_operation_iterator_t itr=begin(), itr_end=end();

      while (itr != itr_end) {
        resource_cost_model_[*itr] = delay_t(r);
        ++itr;
      }
    }

    adjacency_map_t adj_map_;
    adjacency_map_t inv_adj_map_; // all incoming edges of a node //
    delay_cost_model_t delay_cost_model_;
    resource_cost_model_t resource_cost_model_;
    data_op_set_t data_op_set_;
    inplace_op_map_t inplace_op_map_;
}; // class Operation_Dag //

// Using the Cumulative_Resource_State //
typedef mv::lp_scheduler::Cumulative_Resource_State<
  Operation_Dag::resource_t, Operation_Dag::operation_t> resource_state_t;

} // namespace scheduler_unit_tests //

namespace mv {
namespace lp_scheduler {

template<>
struct scheduler_traits<scheduler_unit_tests::Operation_Dag>
  : public scheduler_unit_tests::Operation_Dag {
    ////////////////////////////////////////////////////////////////////////////
    // input graph model and delay model are used from Operation_Dag itself   //
    using scheduler_unit_tests::Operation_Dag::Operation_Dag;

    ////////////////////////////////////////////////////////////////////////////
    // define resource update model //
    typedef scheduler_unit_tests::resource_state_t resource_state_t;

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

}; // specialization for scheduler_unit_tests::dag_t //


} // namespace lp_scheduler //
} // namespace mv //


typedef mv::lp_scheduler::scheduler_traits<scheduler_unit_tests::Operation_Dag>
  unit_testing_scheduler_traits_t;
typedef typename unit_testing_scheduler_traits_t::scheduled_op_t
  unit_testing_scheduled_op_t;

namespace mv {
namespace lp_scheduler {

template<>
struct interval_traits<unit_testing_scheduled_op_t> {
    typedef unit_testing_scheduled_op_t interval_t;
    typedef size_t unit_t;
    // interval traits //
    static unit_t interval_begin(const interval_t& op) { return op.rbegin_; }
    static unit_t interval_end(const interval_t& op) { return op.rend_; }
}; // struct interval_traits //


} // namespace lp_scheduler //
} // namespace mv//



#endif
