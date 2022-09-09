#ifndef TWOD_STRIP_PACKING_LP_H
#define TWOD_STRIP_PACKING_LP_H

#include <cassert>
#include <exception>
#include <glpk.h>
#include <unordered_map>
#include <vector>

#include <scheduler/feasible_scheduler.hpp>


namespace mv {

template<typename dag_concept_t>
struct dag_traits {
  typedef int dag_t;
  typedef int operation_t; // same as node in graph we call it operation_t //
  typedef int const_operation_iterator_t; // const iterator over nodes //
  typedef int unit_t;

  // iterator v \in V //
  static const_operation_iterator_t operations_begin(const dag_t&);
  static const_operation_iterator_t operations_end(const dag_t&);

  // Given v \in V , iterator over { u | (v, u) \in E } 
  static const_operation_iterator_t outgoing_operations_begin(const dag_t&,
        const operation_t&);
  static const_operation_iterator_t outgoing_operations_end(const dag_t&,
        const operation_t&);

  // Given v \in V , iterator over { u | (u, v) \in E } 
  static const_operation_iterator_t incoming_operations_begin(const dag_t&,
      const operation_t&);
  static const_operation_iterator_t incoming_operations_end(const dag_t&,
        const operation_t&);

  static unit_t width(const dag_t&, const operation_t&);
  static unit_t height(const dag_t&, const operation_t&);

  static operation_t create_new_node(dag_t&, const std::string& node_name);
  static void add_edge(const operation_t& src, const operation_t& sink);

}; // struct dag_traits //

template<typename R>
struct rectangle_traits {
  typedef R rect_t;
  typedef int unit_t;
  static unit_t width(const rect_t&);
  static unit_t height(const rect_t&);
  static unit_t llx(const rect_t&);
  static unit_t lly(const rect_t&);
  static void xinterval(const rect_t&, unit_t& xmin, unit_t& xmax);
  static void yinterval(const rect_t&, unit_t& ymin, unit_t& ymax);
  static void create_rect(const unit_t& xmin, const unit_t& ymin,
      const unit_t& width, const unit_t& height, rect_t& rect);
}; // struct rectangle_traits //


struct Overlap_Util {

  template<typename T>
  inline static bool overlapping_rectangles(const T& a, const T& b) {
    typedef rectangle_traits<T> traits;
    typename traits::unit_t a_beg, a_end, b_beg, b_end;


    traits::xinterval(a, a_beg, a_end);
    traits::xinterval(b, b_beg, b_end);

    if (!overlapping_intervals(a_beg, a_end, b_beg, b_end )) { return false; }

    traits::yinterval(a, a_beg, a_end);
    traits::yinterval(b, b_beg, b_end);

    if (!overlapping_intervals(a_beg, a_end, b_beg, b_end )) { return false; }

    return true;
  }

  template<typename T>
  inline static bool overlapping_rectangles_without_touching(
      const T& a, const T& b) {
    typedef rectangle_traits<T> traits;
    typename traits::unit_t a_beg, a_end, b_beg, b_end;


    traits::xinterval(a, a_beg, a_end);
    traits::xinterval(b, b_beg, b_end);

    if (!overlapping_intervals_without_touching(a_beg, a_end, b_beg, b_end )) {
      return false;
    }

    traits::yinterval(a, a_beg, a_end);
    traits::yinterval(b, b_beg, b_end);

    if (!overlapping_intervals_without_touching(a_beg, a_end, b_beg, b_end )) {
      return false;
    }

    return true;
  }

  template<typename Unit>
  inline static bool overlapping_intervals(const Unit& a_beg, const Unit& a_end,
      const Unit& b_beg, const Unit& b_end) {
    return std::max(a_beg, b_beg) <= std::min(a_end, b_end);
  }

  template<typename Unit>
  inline static bool overlapping_intervals_without_touching(
      const Unit& a_beg, const Unit& a_end,
      const Unit& b_beg, const Unit& b_end) {
    return std::max(a_beg, b_beg) < std::min(a_end, b_end);
  }

}; // struct Overlap_Util //

struct Scheduler_DAG_Util {

  template<typename T>
  struct noop_output_iterator_t {
    void operator++() {}
    T& operator*() { return in_; }
    T in_;
  };

  // If the scheduler introduced any spills in the data flow we need to 
  // construct new DAG with new spilled operations and edges. Additionally
  // the caller can pass an output iterator to get an updated schedule with
  // new operation names.
  //
  // Precondition: all nodes are uniquely identified by their name.
  // returns total number of edit operations. 
  template<typename ScheduleInfoIterator, typename SchedulerTraits,
    typename scheduled_op_info_t=typename ScheduleInfoIterator::value_type,
    typename UpdatedScheduleInfoOutputIterator
        = noop_output_iterator_t<scheduled_op_info_t> >
  static size_t rebuild_data_flow_dag_from_schedule(
      typename SchedulerTraits::dag_editor_t& input_dag_editor,
      ScheduleInfoIterator in_sched_begin,
      ScheduleInfoIterator in_sched_end,
      UpdatedScheduleInfoOutputIterator output = 
        UpdatedScheduleInfoOutputIterator()) {

    typedef SchedulerTraits traits;
    typedef typename traits::dag_t dag_t;
    typedef typename traits::resource_t resource_t;
    typedef typename dag_t::operation_t operation_t;
    typedef typename dag_t::const_operation_iterator_t
        const_operation_iterator_t;
    typedef std::unordered_map<operation_t, std::pair<operation_t, size_t> >
      last_spilled_op_map_t;
    last_spilled_op_map_t last_spilled_read, last_spilled_write;

    const dag_t& input_dag = traits::get_dag(input_dag_editor);
    size_t edit_count = 0UL;
    // Single pass data flow rebuilding method:
    //
    // SPILLED WRITE:
    //   1. [NODE]: Create a new node and update last_spilled_write
    //   2. [EDGE]: add edge between last_spilled_read and this new spilled
    //              write.
    //
    // SPILLED READ: 
    //  1. [NODE]: Create a new node update last_spilled_read
    //  2. [EDGE]: 
    //
    //
    // ORIGINAL: is parent of this op spilled ? (last_spilled_write.find(op))
    //  IF YES:
    //    1. [EDGE]: look up last_spilled_read and add an edge.

    bool is_spilled_read=false, is_spilled_write=false;
    for (;in_sched_begin != in_sched_end; ++in_sched_begin) {
      const scheduled_op_info_t &op_info = *in_sched_begin;
      scheduled_op_info_t updated_op_info = op_info;
      operation_t curr_op = op_info.op_;

      if (!strcmp(op_info.op_type_name(), "ORIGINAL") ){
        // any parent spilled ? : atleast one spilled write and read. //
        for (const_operation_iterator_t
              pitr = traits::incoming_operations_begin(input_dag, curr_op);
              pitr != traits::incoming_operations_end(input_dag, curr_op); 
              ++pitr) {
          operation_t pop = *pitr;
          auto itr = last_spilled_read.find(pop);
          if (itr != last_spilled_read.end()) {
            assert(last_spilled_write.find(pop) != last_spilled_write.end());
            // EDIT: data flow from spilled read to this op //
            traits::add_edge(input_dag_editor, (itr->second).first, curr_op); 
            edit_count++;

            // EDIT: remove orig data flow from parent and curr op // 
            traits::remove_edge(input_dag_editor, pop, curr_op);
            edit_count++;
          }
        }
      } else if (
         (is_spilled_read=(!strcmp(op_info.op_type_name(), "SPILLED_READ"))) ||
         (is_spilled_write=(!strcmp(op_info.op_type_name(), "SPILLED_WRITE"))) )
      {

        last_spilled_op_map_t *last_spilled_op_map_ptr =
          is_spilled_read ? &last_spilled_read : &last_spilled_write;
        typename last_spilled_op_map_t::iterator itr =
            last_spilled_op_map_ptr->find(curr_op);
        size_t new_spill_op_id = (itr == last_spilled_op_map_ptr->end()) ?
          0UL : ((itr->second).second)+1UL;
        std::string new_op_name(traits::operation_name(curr_op));
        char buf[128UL];

        sprintf(buf, "%lu", new_spill_op_id);
        new_op_name +=
          (((is_spilled_read) ? "spRd-" : "spWr-") + std::string(buf));

        // EDIT: add new node into the DAG //
        operation_t new_spill_op =
          traits::new_operation(input_dag_editor, new_op_name, curr_op,
           op_info.has_active_resource() ?
           resource_t( (op_info.end_resource()-op_info.begin_resource())+1UL )
            : resource_t(0) );
        edit_count++;

        if (itr == last_spilled_op_map_ptr->end()) {
          last_spilled_op_map_ptr->insert(std::make_pair(curr_op,
                  std::make_pair(new_spill_op, new_spill_op_id) ) );
        } else {
          (itr->second).first = new_spill_op;
          (itr->second).second = new_spill_op_id;
        }


        operation_t prev_spill_op = curr_op;
        if (is_spilled_read) {
          // look for last spilled write and add data flow //
          auto last_spilled_write_itr = last_spilled_write.find(curr_op);
          if (last_spilled_write_itr == last_spilled_write.end()) {
            throw "Invalid Schedule: missing SPILLED WRITE before "
                "SPILLED READ\n";
          }
          prev_spill_op = (last_spilled_write_itr->second).first;
        } else {
          // look for last spilled read and add data flow.
          // NOTE: if there is no previous spilled read then we just add
          // data flow to the original op -- this essentially means this is 
          // the first spill write operation.
          auto last_spilled_read_itr = last_spilled_read.find(curr_op);
          if (last_spilled_read_itr != last_spilled_read.end()) {
            prev_spill_op = (last_spilled_read_itr->second).first;
          }
        }
        // EDIT: add data flow between last write(read) and new read(write)_op 
        traits::add_edge(input_dag_editor, prev_spill_op, new_spill_op); 
        
        // updated scheduled op info //
        updated_op_info.op_ = new_spill_op;
        updated_op_info.reset_as_original_op();

        edit_count++;
      } else {
        throw "Invalid Op type name: expected=[ORIGINAL, S READ, S WRITE] ";
      }
      
      *output = updated_op_info; ++output; 
    }
    return edit_count;
  }

}; // class Scheduler_DAG_Util //


//NOTE: Unit must be a signed type//
template<typename T, typename G=dag_traits<int> >
class Strip_Packing_Linear_Program {

  public:
    ////////////////////////////////////////////////////////////////////////////
    typedef T rtraits;
    typedef typename rtraits::rect_t rect_t;
    typedef typename rtraits::unit_t unit_t;

    typedef G dag_traits;
    typedef typename dag_traits::dag_t dag_t;
    typedef typename dag_traits::operation_t operation_t;
    typedef typename dag_traits::const_operation_iterator_t
        const_operation_iterator_t;

    struct op_placement_gap_t {
      op_placement_gap_t(operation_t op, unit_t x, unit_t y)
        : op_(op), x_(x), y_(y), gap_(0UL) {}
      operation_t op_;
      unit_t x_;
      unit_t y_;
      size_t gap_;
    }; // struct op_placement_gap_t //

    struct coinciding_pair_t {
      size_t rect_idx_i_; // note this is 1 indexing //
      size_t rect_idx_j_;
      coinciding_pair_t(size_t i, size_t j) : rect_idx_i_(i), rect_idx_j_(j) {}
    }; // struct coinciding_pair_t //
    typedef typename std::vector<coinciding_pair_t> coinciding_pairs_t;
    typedef typename coinciding_pairs_t::const_iterator
        coinciding_pairs_iterator_t;

    class strip_packing_exception_t : public std::exception {
      public:
        strip_packing_exception_t(
            const char *exception_str="strip_packing_exception_t:")
          : str_(exception_str) {}

        virtual const char* what() const throw () {
          return str_.c_str();
        }

      private:
        const std::string str_;
    }; // class strip_packing_exception_t //

    struct matrix_index_t {
      matrix_index_t(size_t row_idx, size_t col_idx)
        : row_idx_(row_idx), col_idx_(col_idx) {}
      size_t row_idx_;
      size_t col_idx_;
      
    }; // struct matrix_index_t //

    struct matrix_entry_t {
      matrix_entry_t(const matrix_index_t& index, const unit_t& value)
        : index_(index), value_(value) {}
      matrix_index_t index_;
      unit_t value_;
    }; // struct matrix_entry_t //

    struct constraint_matrix_t {
      // Ax=b , the values of b[i] are encoded as an entry (i,col_dim) //
      typedef std::vector<matrix_entry_t> sparse_matrix_t;
      typedef typename sparse_matrix_t::const_iterator
          const_sparse_matrix_iterator_t;

      sparse_matrix_t entries_;
      size_t row_dim_;
      size_t col_dim_;
      void clear() { entries_.clear(); row_dim_ = 0UL; col_dim_ = 0UL; }
    }; // struct constraint_matrix_t //
    typedef typename constraint_matrix_t::sparse_matrix_t sparse_matrix_t;

    enum direction_e { RIGHT=1, LEFT=2, UP=3, BOTTOM=4 }; // enum direction_e //

    // All the logic of encoding/decoding the LP variables happen in this class.
    class variable_codec_t {

      public:

        ////////////////////////////////////////////////////////////////////////

        class variable_iterator_t {
          public:
            variable_iterator_t(size_t N=0UL, size_t total_var_count=0,
             size_t gap_variables_offset=0UL, size_t curr_var=1UL,
             size_t coinciding_variables_offset=0, size_t coinciding_modulo=6UL,
             coinciding_pairs_t const *coinciding_pairs_ptr=NULL)
           : buf(), N_(N), dir_i_(0UL), dir_j_(N), curr_var_(curr_var),
           curr_dir_(BOTTOM), total_var_count_(total_var_count), 
           gap_variables_offset_(gap_variables_offset),
           coinciding_variables_offset_(coinciding_variables_offset),
           coinciding_modulo_(coinciding_modulo), coinciding_variable_prefix_(),
           coinciding_pairs_ptr_(coinciding_pairs_ptr) {
             setup_coinciding_variable_prefixes();
           }

            bool operator==(const variable_iterator_t& o) const {
              return reached_end() && o.reached_end();
            }

            bool operator!=(const variable_iterator_t& o) const {
              return !(*this == o);
            }

            //Precondition: reached_end() is  true //
            const variable_iterator_t& operator++(void) {
              if ((curr_var_ >= (2UL*N_)) && (dir_i_ < N_) ) {
                if (curr_dir_ == 4 /*BOTTOM*/){
                  // move to a new cell //
                  if ( (dir_j_ == N_) ) {
                    ++dir_i_;
                    dir_j_ = dir_i_+1U;
                  } else {
                    ++dir_j_;
                  }
                  curr_dir_ = direction_e(1);
                } else {
                  // move within the cell //
                  int temp = curr_dir_;
                  ++temp;
                  curr_dir_ = direction_e(temp);
                }
              }
              ++curr_var_;
            }

            char const * operator*() {
              if (curr_var_ <= (2UL*N_)) {
                sprintf(buf, "%s_%lu", (curr_var_%2UL) ? "x" : "y",
                    ((curr_var_/2UL)+(curr_var_%2UL)));
              } else if (curr_var_ == total_var_count_) {
                sprintf(buf, "%s", "z_H");
              } else if ( (curr_var_ > gap_variables_offset_) && 
                  (curr_var_ <= coinciding_variables_offset_) ) {
                sprintf(buf, "g_%lu", (curr_var_-gap_variables_offset_));
              } else if ( (curr_var_ > coinciding_variables_offset_) &&
                  (curr_var_ < total_var_count_) ) {
                size_t delta = (curr_var_ - coinciding_variables_offset_)-1UL;
                size_t pair_idx = (delta/coinciding_modulo_);

                if ( !coinciding_pairs_ptr_ || 
                      (pair_idx > coinciding_pairs_ptr_->size()) ) {
                  throw strip_packing_exception_t("Coinciding Variable "
                        "Invariant Violation");
                }

                const coinciding_pair_t &cpair =
                    coinciding_pairs_ptr_->at(pair_idx);
                sprintf(buf, "%s%lu_%lu",
                      coinciding_variable_prefix_[delta%coinciding_modulo_],
                      cpair.rect_idx_i_, cpair.rect_idx_j_);
              } else {
                // directional variables //
                sprintf(buf, "t_%c_%lu_%lu",
                    get_dir_label(curr_dir_), dir_i_, dir_j_);
              }
              return buf;
            }

          private:

            void setup_coinciding_variable_prefixes() {
              if (coinciding_modulo_ != 6UL) {
                throw strip_packing_exception_t(
                    "Coinciding module currently is setup to be 6");
              }
              coinciding_variable_prefix_.push_back("abs_x_");
              coinciding_variable_prefix_.push_back("abs_y_");
              coinciding_variable_prefix_.push_back("abs_");
              coinciding_variable_prefix_.push_back("t_abs_x_");
              coinciding_variable_prefix_.push_back("t_abs_y_");
              coinciding_variable_prefix_.push_back("t_coincide_");
            }


            char get_dir_label(direction_e dir) const {
              if (dir == RIGHT) { return 'R'; }
              else if (dir == LEFT) { return 'L'; }
              else if (dir == UP) { return 'U'; }
              else { return 'B'; }
            }

            bool reached_end() const { 
              return !N_ || (total_var_count_ < curr_var_);
            }

            mutable char buf[4096UL];
            size_t N_;
            size_t dir_i_;
            size_t dir_j_;
            size_t curr_var_;
            direction_e curr_dir_;
            size_t total_var_count_;
            size_t gap_variables_offset_;
            size_t coinciding_variables_offset_;
            size_t coinciding_modulo_;
            std::vector<char const *> coinciding_variable_prefix_;
            coinciding_pairs_t const *coinciding_pairs_ptr_;
        }; // class variable_iterator_t //

        ////////////////////////////////////////////////////////////////////////
        variable_codec_t(const size_t& dim=0UL)
          : N_(dim), total_var_count_(), row_bounds_col_id_(),
            direction_variables_offset_(), gap_variables_offset_(),
            total_direction_variables_(), coinciding_pairs_(),
            coinciding_variables_offset_() {init();}

        void reset(size_t dim) {
          N_ = dim;
          init();
        }

        size_t coinciding_variable_offset() const {
          return coinciding_variables_offset_;
        }

        coinciding_pairs_iterator_t coinciding_pairs_begin() const {
          return coinciding_pairs_.begin();
        }
        coinciding_pairs_iterator_t coinciding_pairs_end() const {
          return coinciding_pairs_.end();
        }



        // Utility function to return the name of the variable from column. 
        // uses only O(1) space and O(log(V)) operations to determine the
        // variable name.
        std::string get_variable_name_from_col_id(size_t input_col_id) const {
          variable_iterator_t vitr(N_, total_var_count_, gap_variables_offset_,
                input_col_id, coinciding_variables_offset_,
                coinciding_variable_modulo()), end;

          if (vitr == end) { return std::string(""); }

          std::string ret_val(*vitr);
          if (ret_val.at(0) != 't') { return ret_val; }

          // For directional variables we need to decode dir_i_, dir_j_ and
          // curr_dir_ values. We will be using a doubling technique to
          // determine these values in O(log(|V|)) time where |V| max number
          // of variables.
          //
          // NOTE: 1 <= i <= N_-1 and i < j for all directional variables
          //

          // 
          // Let f(r) : direction_variable_col_id(r, r+1, UP) 
          //
          // find 1 <= r <= N_-1 s.t :  f(r) <= input_col_id <= f(r+1) 
          // 
          size_t r_beg = 1UL, r_end = N_-1UL, r_probe, probe_col_id;
          size_t dir_i, dir_j, dir_e; 

          // Invariant: r_beg < r_end && f(r_beg) <= input_col_id <= f(r_end) //
          // STEP-1(NARROW DOWN ROW): 
          while ((r_end - r_beg) > 1) {
            r_probe = r_beg + (r_end-r_beg)/2UL; // r_beg <= r_probe < r_end //
            probe_col_id = direction_variable_col_id(r_probe, r_probe+1, RIGHT);
            if (probe_col_id <= input_col_id) {
              r_beg = r_probe;
            } else {
              r_end = r_probe;
            } 
          }

          if ( (r_end != r_beg) && (direction_variable_col_id(
                    r_end, r_end+1, RIGHT) <= input_col_id) ) {
            dir_i = r_end;
          } else {
            dir_i = r_beg;
          }

          // STEP-2(NARROW DOWN COL): 
          size_t c_beg = dir_i + 1UL, c_end = N_, c_probe;
          while ((c_end-c_beg) > 1UL) {
            c_probe = c_beg + (c_end-c_beg)/2UL; // c_beg <= c_probe < c_end //
            probe_col_id = direction_variable_col_id(dir_i, c_probe, RIGHT);
            if (probe_col_id <= input_col_id) {
              c_beg = c_probe;
            } else {
              c_end = c_probe;
            }
          }

          if (direction_variable_col_id(dir_i, c_end, RIGHT) <= input_col_id) {
            dir_j = c_end;
          } else {
            dir_j = c_beg;
          }

          char dir_char[4] = {'R', 'L', 'U', 'B'};
          char buf[4096];

          size_t base_col_id = direction_variable_col_id(1UL, 2UL, RIGHT);
          assert(base_col_id <= input_col_id);
          size_t dir_delta = (input_col_id-base_col_id);

          sprintf(buf, "t_%c_%lu_%lu", dir_char[dir_delta%4UL], dir_i, dir_j);
          return std::string(buf);
        }

        inline bool is_valid() const { return N_ > 0UL; }
        inline size_t total_variable_count() const { return total_var_count_; }
        inline size_t z_H_col_id() const { return total_var_count_; }
        inline size_t row_bounds_col_id() const { return row_bounds_col_id_; }
        inline size_t gap_variable_col_id(size_t i) const {
          return (total_var_count_-1UL) - (N_-i);
        }
        std::pair<size_t, size_t> get_col_ids_of_rectangle(size_t k) const {
          assert(k > 0);
          return std::make_pair( (2UL*k)-1UL, 2UL*k);
        }

        // Precondition : i < j //
        // NOTE: all (N*(N-1))/2 directional variables are encoded in a
        // linear order as follows:
        //
        // Upper triangular matrix:
        // 
        //   (1,1) (1,2) (1,3) ........        (1,N)
        //         (2,2) (2,3) (2,4) (2,5) ... (2,N)      
        //               (3,3) (3,4) (3,5) ... (3,N)
        //                       ...................
        //                             .............
        //                                     .....
        //                                       ...
        //                         (N-1,N-1) (N-1,N)
        //
        //
        //
        // Linear order ignoring diagonal elements:
        // L = { (1,2) , (1,3) ..... (N-1,N) }
        //
        // Given (i,j) s.t i<j what is its linear index ?
        //
        // 1. |L| = 1+2...+N-1 = (N-1)*N)/2 
        // 2. # of elements in the first i-1 rows:
        //      total_elements - elements_in_last_(i-1)_rows which is
        //      ((N-1)*N)/2 - ((N-i)*(N-i+1))/2 
        // 3. Elements before j in the column is j-i-1
        //
        // final answer: ( ((N-1)*N) - (N-i)*(N-i+1) ) / 2 + (j-i-1) + 1 //
        // 
        // E.g. (1,3) : ((N-1)*N) - (N-1)*(N-1+1))/2 + (3-1-1) + 1 = 2 // 
        // E.g. (N-1,N): ((N-1)*N - (N-(N-1))*(N-(N-1)+1))/2 + (N-(N-1)-1) + 1
        //               = ((N-1)*N - 2)/2 + 1 = ((N-1)*N)/2 (total elements) 
        //
        // NOTE: each cell has 4 variables so we have to scale by 4 //
        //
        // 4 * (sum(N-1) - sum(N-i)) + 4*(j-i-1) + dir_variable_number //

        inline size_t direction_variable_col_id(size_t i, size_t j,
              direction_e dir_indicator) const {

          assert((i>=1) && (i<=N_)); assert((j>=1) && (j<=N_)); assert(j>i);

          size_t dir_vars_in_first_i_rows = (Sigma(N_-1) - Sigma(N_-i));
          size_t dir_vars_in_i_th_row_before_j = (j-i-1);

          size_t index  = 4*( ((Sigma(N_-1) - Sigma(N_-i)) + (j-i-1)) ) +
              dir_indicator;
          assert( index <= (4UL*Sigma(N_-1)) );

          return direction_variables_offset_ + index;
        }

        inline bool coinciding_pair_col_id(size_t i, size_t j, size_t& col_id) {
          size_t offset=1;
          for (const coinciding_pair_t& cpair : coinciding_pairs_) {
            if ((cpair.rect_idx_i_ == i) && (cpair.rect_idx_j_ == j) ) {
              size_t coffset = coinciding_variable_offset();
              col_id = coinciding_variable_offset() + (offset*6UL);
              return true;
            }
            ++offset;
          }
          return false;
        }

        variable_iterator_t begin() const {
          return variable_iterator_t(N_, total_var_count_,
                gap_variables_offset_, 1UL, coinciding_variables_offset_,
                coinciding_variable_modulo(), &coinciding_pairs_);
        }

        variable_iterator_t end() const {
          return variable_iterator_t();
        }

        void add_coinciding_pair(size_t i, size_t j) {
          if ( !((i<j) && (i>=1) && (j<=N_)) ) { 
            throw strip_packing_exception_t("Coinciding pair "
                  "invariant violation");
          }
          coinciding_pairs_.push_back(coinciding_pair_t(i, j));
        }

      private:

        // \Sigma_{t=1}^{t=i}  t = (i*(i+1))/2 //
        inline size_t Sigma(size_t i) const {
          return i%2UL ? ((i+1UL)/2UL)*i : (i/2UL)*(i+1) ;
        }

        void init() {
          // pvars : placement ; dvars : direction; gvars : gap;
          // cvars : coinciding 
          //
          // Encoding order:
          //
          // <pvars> <dvars> <gvars> <cvars> z_H //
          //
          // 
          total_direction_variables_ =
              4UL * ( (N_%2UL) ? ((N_-1)/2UL)*N_ : (N_/2UL)*(N_-1) );

          total_var_count_ = (2UL * N_) + total_direction_variables_ + N_ +
              + total_coinciding_variable_count() + 1UL;
          
          // n gap variables //g_i //
          gap_variables_offset_ = total_var_count_-
              (N_ + total_coinciding_variable_count() +1UL); 

          coinciding_variables_offset_ =
            total_var_count_ - (total_coinciding_variable_count() + 1UL);
          row_bounds_col_id_ = total_var_count_ + 1UL;
          direction_variables_offset_ = 2*N_;
        }


        inline size_t total_coinciding_variable_count() const {
          /////////////////////////////////////////////////////////////////
          // per pair analysis
          //   abs_x_i_j = | x_i - x_j | (def)
          //   abs_y_i_j = | y_i - y_j | (def)
          //   abs_i_j = abs_x_i_j + abs_y_i_j (def)
          //
          //   Variable synthesis using following constraints:
          // 
          //   1. x_i - x_j <= abs_x_i_j 
          //   2. x_j - x_i <= abs_x_i_j
          //   3. x_j - x_j <=     t_abs_x_i_j*2M  - abs_x_i_j
          //   4. x_i - x_j <=  (1-t_abs_x_i_j)*2M - abs_x_i_j
          //
          //   5. y_i - y_j <= abs_y_i_j 
          //   6. y_j - y_i <= abs_y_i_j
          //   7. y_j - y_j <=     t_abs_y_i_j*2M  - abs_y_i_j
          //   8. y_i - y_j <=  (1-t_abs_y_i_j)*2M - abs_y_i_j
          //   
          //   9.  abs_i_j - abs_x_i_j - abs_y_i_j <= 0
          //  10  -abs_i_j + abs_x_i_j + abs_y_i_j <= 0
          //
          //
          //  Coinciding indictator: 
          //  11. -abs_i_j <= t_coincide_i_j*2M - 1
          //  12.  abs_i_j <= 2M - t_coincide_i_j*2M
          //
          // Total Variables (6 variables and 12 constraints):
          // { abs_x_i_j, abs_y_i_j, abs_i_j, t_abs_x_i_j, t_abs_y_i_j,
          //    t_concide_i_j }
          //
          //////////////////////////////////////////////////////////////////
          return (6UL*(coinciding_pairs_.size()));
        }
        inline size_t coinciding_variable_modulo() const { return 6UL; }

        ////////////////////////////////////////////////////////////////////////
        size_t N_; // dimension //
        size_t total_var_count_;
        size_t row_bounds_col_id_;
        size_t direction_variables_offset_;
        size_t gap_variables_offset_;
        size_t total_direction_variables_;
        coinciding_pairs_t coinciding_pairs_;
        size_t coinciding_variables_offset_;
    }; // class variable_codec_t //


    class row_constraint_iterator_t {
      public:
        ////////////////////////////////////////////////////////////////////////
        typedef typename constraint_matrix_t::const_sparse_matrix_iterator_t
            const_sparse_matrix_iterator_t;


        struct variable_name_coefficient_iterator_t {

          variable_name_coefficient_iterator_t() : col_begin_(), col_end_(),
            vname_(), vcodec_ptr_() {}

          variable_name_coefficient_iterator_t(
              const_sparse_matrix_iterator_t col_begin,
              const_sparse_matrix_iterator_t col_end,
              variable_codec_t const *vcodec_ptr)
            : col_begin_(col_begin), col_end_(col_end), vname_(),
              vcodec_ptr_(vcodec_ptr) {}

          void operator++() { col_begin_++; }

          bool operator==(const variable_name_coefficient_iterator_t& o) const {
            return reached_end() && o.reached_end();
          }

          bool operator!=(const variable_name_coefficient_iterator_t& o) const {
            return !(*this == o);
          }

          // Precondition: !reached_end() //
          char const * variable_name() const {
            vname_ = vcodec_ptr_->get_variable_name_from_col_id(
                (col_begin_->index_).col_idx_ );
            return vname_.c_str();
          }

          size_t column_id() const { return (col_begin_->index_).col_idx_; }

          // Precondition: !reached_end() //
          unit_t coefficient_value() const {
            return col_begin_->value_;
          }

          private:

          bool reached_end() const { return col_begin_ == col_end_; }

          const_sparse_matrix_iterator_t col_begin_;
          const_sparse_matrix_iterator_t col_end_;
          mutable std::string vname_;
          variable_codec_t const * vcodec_ptr_;
        }; // struct variable_name_coefficient_iterator_t //
        ////////////////////////////////////////////////////////////////////////

        row_constraint_iterator_t() : row_itr_(), row_itr_end_(),
          matrix_itr_end_() {}

        row_constraint_iterator_t(const constraint_matrix_t& matrix,
            const variable_codec_t& vcodec)
          : row_itr_(), row_itr_end_(), matrix_itr_end_(), vcodec_ptr_(&vcodec)
        {
          const sparse_matrix_t & smatrix = matrix.entries_;
          row_itr_ = smatrix.begin();
          row_itr_end_ = row_itr_;
          matrix_itr_end_ = smatrix.end();

          // initialize //
          ++(*this);
        }

        //Precondition:: reached_end() is false //
        void operator++() {
          // Invariant: row_itr_end_ points to the end of the last row//
          row_itr_ = row_itr_end_;
          while ( !reached_end() &&
              ((*row_itr_end_).index_.row_idx_ == (*row_itr_).index_.row_idx_)){
            ++row_itr_end_;
          }
        }

        bool operator==(const row_constraint_iterator_t& o) const {
          return reached_end() && o.reached_end();
        }

        bool operator!=(const row_constraint_iterator_t& o) const {
          return !(*this == o);
        }

        variable_name_coefficient_iterator_t col_begin() const {
          return variable_name_coefficient_iterator_t(row_itr_, row_itr_end_,
                vcodec_ptr_);
        }

        variable_name_coefficient_iterator_t col_end() const {
          return variable_name_coefficient_iterator_t();
        }

      private:

        bool reached_end() const { return (row_itr_end_ == matrix_itr_end_); }

        ////////////////////////////////////////////////////////////////////////
        const_sparse_matrix_iterator_t row_itr_;
        const_sparse_matrix_iterator_t row_itr_end_;
        const_sparse_matrix_iterator_t matrix_itr_end_;
        variable_codec_t const * vcodec_ptr_;
    }; // class row_constraint_iterator_t //


    typedef typename variable_codec_t::variable_iterator_t variable_iterator_t;

    //TODO(vamsikku): use operation_hash //
    typedef std::unordered_map<operation_t, size_t> node_map_t;


    ////////////////////////////////////////////////////////////////////////////

    Strip_Packing_Linear_Program()
      : W_(), H_(), N_(), widths_(), heights_(), constraint_matrix_(),
        variable_codec_(), node_map_() {}

    //////////////////////////// VARIABLE ENCODING/////////////////////////////
    /*

       // N_ = number of bins //
       // column order
        x_1 y_1 x_2 y_2 ..... x_n y_n t_1_2_R t_1_2_L t_1_2_U t_1_2_B ... z_H

       // Index of direction control variable per pair of rectangles
       // r_i , r_j (i < j):
       //
       //  ((n(n-1))/2 - ((n-i+1)(n-i))/2) + ((j-i)-1)*4 + 1 //
       //

     */
    ////////////////////////////////////////////////////////////////////////////
    template<typename RectangleIterator>
    void reset(RectangleIterator begin, RectangleIterator end,
        unit_t strip_width, unit_t height_bound) {
      W_ = strip_width;
      H_ = height_bound;
      widths_.clear();
      heights_.clear();

      for (; begin!=end; ++begin) {
        rect_t const &rect = *begin;
        widths_.push_back(rtraits::width(rect));
        heights_.push_back(rtraits::height(rect));
      }
      N_ = widths_.size();
      init();
    }

    void reset_from_dag(const dag_t& input_dag,
          size_t strip_width, size_t height_bound) {

      W_ = strip_width;
      H_ = height_bound;
      widths_.clear();
      heights_.clear();

      const_operation_iterator_t node_itr =
          dag_traits::operations_begin(input_dag);
      const_operation_iterator_t node_itr_end =
          dag_traits::operations_end(input_dag);

      size_t node_id = 1;
      for (; node_itr != node_itr_end; ++node_itr) {
        const operation_t& node= *node_itr;
        widths_.push_back(dag_traits::width(input_dag, node));
        heights_.push_back(dag_traits::height(input_dag, node));
        node_map_[node] = node_id++;
      }

      N_ = widths_.size();
      init();
    }

    //NOTE: this must be called before constraint matrix generation //
    template<typename CoincidingPairIterator>
    void add_coinciding_pairs_and_update_variables(
        CoincidingPairIterator cbegin, CoincidingPairIterator cend) {
      for (;cbegin != cend; ++cbegin) {
        const coinciding_pair_t& cpair = *cbegin;
        variable_codec_.add_coinciding_pair(
            cpair.rect_idx_i_, cpair.rect_idx_j_);
      }
      init();
    }

    size_t dimension() const { return N_; }

    void generate_constraint_matrix() {
      constraint_matrix_.row_dim_ = 0UL;
      constraint_matrix_.entries_.clear();
      constraint_matrix_.col_dim_ = total_variable_count();

      constraint_matrix_.row_dim_ =
          generate_contiguous_resource_constraints(constraint_matrix_.row_dim_);
      constraint_matrix_.row_dim_ =
          generate_width_height_constraints(constraint_matrix_.row_dim_);
      constraint_matrix_.row_dim_ = generate_coinciding_rectangle_constraints(
              constraint_matrix_.row_dim_);
    }

    void generate_constraint_matrix(const dag_t& input_dag,
          bool ignore_precedence_constraints=false,
          bool ignore_producer_consumer_constraints=true) {
      // Non-Overlapping constraints //
      generate_constraint_matrix();
      if (!ignore_precedence_constraints) {
        constraint_matrix_.row_dim_ = generate_precedence_constraints(input_dag,
            constraint_matrix_.row_dim_);
      }

      if (!ignore_producer_consumer_constraints) {
        constraint_matrix_.row_dim_ = generate_producer_consumer_constraints(
            input_dag, constraint_matrix_.row_dim_);
      }
    }

    const constraint_matrix_t& get_constraint_matrix() const {
      return constraint_matrix_;
    }

    const unit_t& get_width() const { return W_; }
    const unit_t& get_height() const { return H_; }

    //NOTE: all col_id's are 1 indexed //
    std::string get_variable_name_from_col_id(size_t input_col_id) const {
     return variable_codec_.get_variable_name_from_col_id(input_col_id);
    }

    row_constraint_iterator_t constraints_begin() const {
      return row_constraint_iterator_t(constraint_matrix_, variable_codec_);
    }

    row_constraint_iterator_t constraints_end() const {
      return row_constraint_iterator_t();
    }


    variable_iterator_t variable_iterator_begin() const {
      return variable_codec_.begin();
    } 

    variable_iterator_t variable_iterator_end() const {
      return variable_codec_.end();
    }

  private:

    size_t generate_width_height_constraints(size_t running_row_index=0UL) {
      // x_i  <= W-w_i //
      size_t row_idx = running_row_index;
      size_t bounds_col_index = get_row_bounds_col_id();
      for (size_t i=1; i<=N_; i++, row_idx++) {
        auto x_y_i = get_col_ids_of_rectangle(i);
        size_t x_i_col_idx = x_y_i.first;
        constraint_matrix_.entries_.push_back(
          matrix_entry_t(matrix_index_t(row_idx, x_i_col_idx), unit_t(1)) );
        constraint_matrix_.entries_.push_back(
            matrix_entry_t(matrix_index_t(row_idx, bounds_col_index),
              unit_t(W_ - widths_[i-1])) );
      }

      // y_i -z_h <= -h_i //
      size_t z_h_col_idx = get_z_H_column_id();
      for (size_t i=1; i<=N_; i++, row_idx++) {
        auto x_y_i = get_col_ids_of_rectangle(i);
        size_t y_i_col_idx = x_y_i.second;
        size_t g_i_col_idx = get_gap_variable_col_id(i);
        // y_i - z_H <= h_i //
        constraint_matrix_.entries_.push_back(
            matrix_entry_t(matrix_index_t(row_idx, y_i_col_idx), unit_t(1)) );

        constraint_matrix_.entries_.push_back(
            matrix_entry_t(matrix_index_t(row_idx, g_i_col_idx), unit_t(1)) );

        constraint_matrix_.entries_.push_back(
            matrix_entry_t(matrix_index_t(row_idx, z_h_col_idx), unit_t(-1)) );
        constraint_matrix_.entries_.push_back(
            matrix_entry_t(matrix_index_t(row_idx, bounds_col_index),
                unit_t(-1*heights_[i-1])) );
      }

      // z_h <= H_ //

      constraint_matrix_.entries_.push_back(
          matrix_entry_t(matrix_index_t(row_idx, z_h_col_idx),
              unit_t(1)) );
      constraint_matrix_.entries_.push_back(
          matrix_entry_t(matrix_index_t(row_idx, bounds_col_index), 
              unit_t(H_)) );

      row_idx++;
      return row_idx;
    }

    //TODO(vamsikku): this creates a constraints for every pair of rectangles
    //but some of these constraints may be redundant.
    size_t generate_contiguous_resource_constraints(
          size_t running_row_index=0UL) {

      size_t row_bound_col_idx = get_row_bounds_col_id();
      size_t row_idx = running_row_index;

      for (size_t i=1; i<=N_; i++) {
        auto x_y_i = get_col_ids_of_rectangle(i);
        size_t x_i_col_idx = x_y_i.first, y_i_col_idx = x_y_i.second;
        if (widths_[i-1] == 0UL) { continue; }

        for (size_t j=i+1; j<=N_; j++) {
          if (widths_[j-1] == 0UL) { continue; }

          // Introduce these 5 constraints for each pair //
          //  x_i - x_j - 2W*t_R <= -w_i
          //  x_j - x_i - 2W*t_L <= -w_j
          //  y_i - y_j - 2H*t_B + g_i <= -h_i
          //  y_j - y_i - 2H*t_U + g_j <= -h_j
          //  t_R + t_L + t_B + t_U <= 3

          auto x_y_j = get_col_ids_of_rectangle(j);
          size_t x_j_col_idx = x_y_j.first, y_j_col_idx = x_y_j.second;

          size_t dir_col_index[4UL] = {
            get_col_id_UP(i, j), get_col_id_LEFT(i, j),
          get_col_id_BOTTOM(i, j), get_col_id_RIGHT(i, j)
          };

          size_t constraint_col_indicies[3UL];
          unit_t row_bound_value;
          for (size_t r=0; r<4UL; r++, ++row_idx) {
            if ( r == 0) {
              constraint_col_indicies[0] = x_y_i.first;
              constraint_col_indicies[1] = x_y_j.first;
              row_bound_value = unit_t(-1)*(widths_[i-1]);
            } else if (r == 1) {
              constraint_col_indicies[0] = x_y_j.first;
              constraint_col_indicies[1] = x_y_i.first;
              row_bound_value = unit_t(-1)*(widths_[j-1]);
            } else if (r == 2) {
              constraint_col_indicies[0] = x_y_i.second;
              constraint_col_indicies[1] = x_y_j.second;
              row_bound_value = unit_t(-1)*(heights_[i-1]);
            } else {
              constraint_col_indicies[0] = x_y_j.second;
              constraint_col_indicies[1] = x_y_i.second;
              row_bound_value = unit_t(-1)*(heights_[j-1]);
            }
            constraint_col_indicies[2] = dir_col_index[r];

            constraint_matrix_.entries_.push_back(
                matrix_entry_t(
                  matrix_index_t(row_idx, constraint_col_indicies[0]),
                  unit_t(1)) );

            constraint_matrix_.entries_.push_back(
                matrix_entry_t(
                  matrix_index_t(row_idx, constraint_col_indicies[1]),
                  unit_t(-1)) );

            constraint_matrix_.entries_.push_back(
                matrix_entry_t(
                  matrix_index_t(row_idx, constraint_col_indicies[2]),
                    (r < 2UL) ? unit_t(-2*W_) : unit_t(-2*H_) ) );

            if (r >= 2) {
              // add gap variables //
              size_t gap_col_id = (r==2) ? get_gap_variable_col_id(i) :
                get_gap_variable_col_id(j);
              constraint_matrix_.entries_.push_back(
                matrix_entry_t(
                  matrix_index_t(row_idx, gap_col_id), unit_t(1))
                );
            }

            constraint_matrix_.entries_.push_back(
                matrix_entry_t(matrix_index_t(row_idx, row_bound_col_idx),
                    row_bound_value) );
          } // foreach of the 4 constraints //


          for (size_t dir=0; dir<4; ++dir) {
            constraint_matrix_.entries_.push_back(
                matrix_entry_t(matrix_index_t(row_idx, dir_col_index[dir]),
                    unit_t(1) ));
          }

          // if (i,j) is a coinciding pair exists add -t_coincde_i_j //
          size_t coincide_col_id;
          if (variable_codec_.coinciding_pair_col_id(i, j, coincide_col_id)) {
            constraint_matrix_.entries_.push_back(
                matrix_entry_t(matrix_index_t(row_idx, coincide_col_id),
                    unit_t(-1) ));
          }

          constraint_matrix_.entries_.push_back(
                matrix_entry_t(matrix_index_t(row_idx, row_bound_col_idx),
                    unit_t(3)) );

          ++row_idx;
        } // foreach j > i //
      } // foreach i //
      return row_idx;
    }
    
    size_t generate_precedence_constraints(const dag_t& input_dag,
        size_t running_row_index=0UL) {
      const_operation_iterator_t node_itr =
          dag_traits::operations_begin(input_dag);
      const_operation_iterator_t node_itr_end =
          dag_traits::operations_end(input_dag);

      size_t row_idx = running_row_index;
      size_t bounds_col_id = get_row_bounds_col_id();

      for (; node_itr!=node_itr_end; ++node_itr, ++row_idx) {
        const operation_t& curr_node = *node_itr;
        size_t curr_node_id = node_map_[curr_node];
        auto curr_node_col_ids = get_col_ids_of_rectangle(curr_node_id);
        size_t y_curr_node_col_id = curr_node_col_ids.second;
        // foreach node v \in G(V,E)
        //    let W = { w | (w,v) \in E }
        //
        //     y_v - y_w >= height(w)
        //
        //     width = resource , height = time
        const_operation_iterator_t pitr =
            dag_traits::incoming_operations_begin(input_dag, curr_node);
        const_operation_iterator_t pitr_end = 
            dag_traits::incoming_operations_end(input_dag, curr_node);

        for (; pitr!=pitr_end; ++pitr) {
          const operation_t& parent_node = *pitr;
          size_t parent_node_id = node_map_[parent_node];
          auto parent_node_col_ids = get_col_ids_of_rectangle(parent_node_id);

          //     y_{curr_node} - y_{parent_node} >= height(parent_node)  //

          //     -> y_{parent_node} - y_{curr_node} <= -height(parent_node) //
          size_t y_parent_node_col_id = parent_node_col_ids.second;

          constraint_matrix_.entries_.push_back(
           matrix_entry_t(matrix_index_t(row_idx, y_parent_node_col_id),
              unit_t(1))
          );

          constraint_matrix_.entries_.push_back(
           matrix_entry_t(matrix_index_t(row_idx, y_curr_node_col_id),
              unit_t(-1))
          );

          constraint_matrix_.entries_.push_back(
              matrix_entry_t(matrix_index_t(row_idx, bounds_col_id),
               (unit_t(-1)*height(parent_node_id)) )
          );

          ++row_idx;
        }
      }

      return row_idx;
    }

    size_t generate_producer_consumer_constraints(const dag_t& input_dag,
        size_t running_row_index=0UL) {
      const_operation_iterator_t node_itr =
          dag_traits::operations_begin(input_dag);
      const_operation_iterator_t node_itr_end =
          dag_traits::operations_end(input_dag);

      size_t row_idx = running_row_index;
      size_t bounds_col_id = get_row_bounds_col_id();

      for (; node_itr!=node_itr_end; ++node_itr, ++row_idx) {
        const operation_t& curr_node = *node_itr;
        size_t curr_node_id = node_map_[curr_node];

        auto curr_node_col_ids = get_col_ids_of_rectangle(curr_node_id);
        size_t y_curr_node_col_id = curr_node_col_ids.second;
        size_t g_curr_node_col_id = get_gap_variable_col_id(curr_node_id);
        // foreach node v \in G(V,E)
        //    let W = { w | (v,w) \in E }
        //
        //     y_v + (height(v) + g_v) >= y_w + height(w)
        //    
        //   -> -y_v -(height(v) + g_v) <= -y_w -height(w)
        //   -> -y_v + y_w - g_v <= (height(v) - height(w))
        //
        //   v = curr_node , w = child_node //
        const_operation_iterator_t citr =
            dag_traits::outgoing_operations_begin(input_dag, curr_node);
        const_operation_iterator_t citr_end = 
            dag_traits::outgoing_operations_end(input_dag, curr_node);

        for (; citr!=citr_end; ++citr) {
          const operation_t& child_node = *citr;
          size_t child_node_id = node_map_[child_node];
          auto child_node_col_ids = get_col_ids_of_rectangle(child_node_id);

          //     y_{child_node} - g_{curr_node} <= -height(child_node)  //

          size_t y_child_node_col_id = child_node_col_ids.second;

          constraint_matrix_.entries_.push_back(
           matrix_entry_t(matrix_index_t(row_idx, y_curr_node_col_id),
              unit_t(-1))
          );

          constraint_matrix_.entries_.push_back(
           matrix_entry_t(matrix_index_t(row_idx, y_child_node_col_id),
              unit_t(1))
          );

          constraint_matrix_.entries_.push_back(
           matrix_entry_t(matrix_index_t(row_idx, g_curr_node_col_id),
              unit_t(-1))
          );

          constraint_matrix_.entries_.push_back(
              matrix_entry_t(matrix_index_t(row_idx, bounds_col_id),
               (height(curr_node_id) - height(child_node_id)) )
          );

          ++row_idx;
        }
      }

      return row_idx;
    }

    size_t generate_coinciding_rectangle_constraints(
        size_t running_row_index=0UL) {

      size_t row_idx = running_row_index;
      size_t col_id_base = variable_codec_.coinciding_variable_offset() + 1UL;
      size_t bounds_col_id = get_row_bounds_col_id();
      unit_t M = std::max(W_, H_);
      for (auto cpair_itr=variable_codec_.coinciding_pairs_begin();
            cpair_itr!=variable_codec_.coinciding_pairs_end(); ++cpair_itr) {
        const coinciding_pair_t& cpair = *cpair_itr;

        auto x_y_i = get_col_ids_of_rectangle(cpair.rect_idx_i_);
        auto x_y_j = get_col_ids_of_rectangle(cpair.rect_idx_j_);
        size_t abs_x_i_j = col_id_base, abs_y_i_j = col_id_base+1UL,
               abs_i_j = col_id_base+2UL, t_abs_x_i_j = col_id_base+3UL,
               t_abs_y_i_j = col_id_base+4UL, t_coincide_i_j = col_id_base+5UL;
        size_t x_i = x_y_i.first, y_i = x_y_i.second;
        size_t x_j = x_y_j.first, y_j = x_y_j.second;

          // 
          //   1.>  x_i - x_j <= abs_x_i_j 
          //        x_i - x_j - abs_x_i_j <= 0
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, x_i), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, x_j), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_x_i_j), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, bounds_col_id), unit_t(0)));


        ++row_idx;
          //   2. x_j - x_i <= abs_x_i_j
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, x_j), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, x_i), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_x_i_j), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, bounds_col_id), unit_t(0)));

        ++row_idx;
          //   3. x_j - x_i <=     t_abs_x_i_j*2M  - abs_x_i_j
          //      x_j - x_i - 2*M t_abs_x_i_j + abs_x_i_j <= 0
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, x_j), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, x_i), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, t_abs_x_i_j), unit_t(-2*M)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_x_i_j), unit_t(1)) );
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, bounds_col_id), unit_t(0)));

        ++row_idx;
          //   4. x_i - x_j <=  (1-t_abs_x_i_j)*2M - abs_x_i_j
          //      x_i - x_j +2M*t_abs_x_i_j + abs_x_i_j <= 2M
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, x_i), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, x_j), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, t_abs_x_i_j), unit_t(2*M)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_x_i_j), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, bounds_col_id),
                unit_t(2*M)));
        ++row_idx;








          //   5. y_i - y_j <= abs_y_i_j 
          //   6. y_j - y_i <= abs_y_i_j
          //   7. y_j - y_j <=     t_abs_y_i_j*2M  - abs_y_i_j
          //   8. y_i - y_j <=  (1-t_abs_y_i_j)*2M - abs_y_i_j


          //   5. y_i - y_j <= abs_y_i_j 
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, y_i), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, y_j), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_y_i_j), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, bounds_col_id), unit_t(0)));


        ++row_idx;


          //   6. y_j - y_i <= abs_y_i_j
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, y_j), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, y_i), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_y_i_j), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, bounds_col_id), unit_t(0)));

        ++row_idx;

          //   7. y_j - y_j <=     t_abs_y_i_j*2M  - abs_y_i_j
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, y_j), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, y_i), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, t_abs_y_i_j), unit_t(-2*M)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_y_i_j), unit_t(1)) );
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, bounds_col_id), unit_t(0)));

        ++row_idx;

          //   8. y_i - y_j <=  (1-t_abs_y_i_j)*2M - abs_y_i_j
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, y_i), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, y_j), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, t_abs_y_i_j), unit_t(2*M)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_y_i_j), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, bounds_col_id),
                unit_t(2*M)));
        ++row_idx;

          //   
          //   9.  abs_i_j - abs_x_i_j - abs_y_i_j <= 0
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_i_j), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_x_i_j), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_y_i_j), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, bounds_col_id), unit_t(0)));


        ++row_idx;
          //  10  -abs_i_j + abs_x_i_j + abs_y_i_j <= 0
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_i_j), unit_t(-1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_x_i_j), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, abs_y_i_j), unit_t(1)));
        constraint_matrix_.entries_.push_back(
            matrix_entry_t( matrix_index_t(row_idx, bounds_col_id), unit_t(0)));

        ++row_idx;

        col_id_base += 6UL;
      }

      return row_idx;
    }

  public:

    // Returns the col index of variables x_k , y_k corresponding to rect r_k //
    std::pair<size_t, size_t> get_col_ids_of_rectangle(size_t k) const {
      return variable_codec_.get_col_ids_of_rectangle(k);
    }

    size_t total_variable_count() const { 
      return variable_codec_.total_variable_count();
    }

    size_t get_row_bounds_col_id() const {
      return variable_codec_.row_bounds_col_id();
    }
    size_t get_z_H_column_id() const { return total_variable_count(); }
    size_t get_gap_variable_col_id(size_t i) const {
      return variable_codec_.gap_variable_col_id(i);
    }

    /////////////////// Direction variable col. indexes ////////////////////////
    size_t get_col_id_UP(size_t i, size_t j) const {
      return variable_codec_.direction_variable_col_id(i, j, UP);
    }

    size_t get_col_id_LEFT(size_t i, size_t j) const {
      return variable_codec_.direction_variable_col_id(i, j, LEFT);
    }

    size_t get_col_id_RIGHT(size_t i, size_t j) const { 
      return variable_codec_.direction_variable_col_id(i, j, RIGHT);
    }

    size_t get_col_id_BOTTOM(size_t i, size_t j) const { 
      return variable_codec_.direction_variable_col_id(i, j, BOTTOM);
    }
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    // Following are two methods used to derive auxiliary variables from
    // placements.
    // 
    // NOTE: both rectangles must have width and height >= 1 //
    static void derive_direction_variables_from_placement_pair(
        const rect_t& r_i, const rect_t& r_j, unit_t& t_R, unit_t& t_L,
          unit_t& t_U, unit_t& t_B, bool swap_up_and_right=false) {
      unit_t w_i = rtraits::width(r_i), h_i = rtraits::height(r_i);
      unit_t w_j = rtraits::width(r_j), h_j = rtraits::height(r_j);
      unit_t x_i, y_i, x_j, y_j;

      x_i = rtraits::llx(r_i);
      x_j = rtraits::llx(r_j);
      y_i = rtraits::lly(r_i);
      y_j = rtraits::lly(r_j);

      t_R = ((x_i + w_i) <= x_j) ? unit_t(0) : unit_t(1);
      t_L = ((x_j + w_j) <= x_i) ? unit_t(0) : unit_t(1);
      t_B = ((y_i + h_i) <= y_j) ? unit_t(0) : unit_t(1);
      t_U = ((y_j + h_j) <= y_i) ? unit_t(0) : unit_t(1);

      if (swap_up_and_right) {
        std::swap(t_U, t_R);
      }
    }

    //NOTE: the order of Placements must be in the same order as the nodes in
    //the dag.
    template<typename InputPlacementIterator,
             typename OutputGapVariableIterator>
    static void derive_gap_variables_from_packing(const dag_t& input_dag,
        InputPlacementIterator pbegin, InputPlacementIterator pend,
        OutputGapVariableIterator output) {

      std::unordered_map<operation_t, op_placement_gap_t> placed_ops;

      const_operation_iterator_t node_itr =
          dag_traits::operations_begin(input_dag);
      const_operation_iterator_t node_itr_end =
          dag_traits::operations_end(input_dag);

      for (; node_itr != node_itr_end; ++node_itr, ++pbegin) {
        if (pbegin == pend) {
          throw strip_packing_exception_t(
              "placement iterator expired before node iterator");
        }
        unit_t px = rtraits::llx(*pbegin);
        unit_t py = rtraits::lly(*pbegin);
        operation_t op = *node_itr;

        auto itr = placed_ops.find(op);
        if (itr != placed_ops.end()) {
          throw strip_packing_exception_t("duplicate node");
        }
        placed_ops.insert(std::make_pair(op, op_placement_gap_t(op, px, py)));
      }


      // Compute gap variables //
      for (node_itr = dag_traits::operations_begin(input_dag);
            node_itr != node_itr_end; ++node_itr, ++output) {

        operation_t curr_op = *node_itr;
        auto itr = placed_ops.find(curr_op);
        if (itr == placed_ops.end()) {
          throw strip_packing_exception_t("missing placed ops entry.");
        }

        op_placement_gap_t &op_placement_gap = itr->second;
        const_operation_iterator_t citr =
          dag_traits::outgoing_operations_begin(input_dag, curr_op);
        const_operation_iterator_t citr_end =
          dag_traits::outgoing_operations_end(input_dag, curr_op);

        unit_t curr_op_end_time =
            op_placement_gap.y_ + delay(input_dag, curr_op);
        unit_t curr_op_orig_end_time = curr_op_end_time;
        
        for (; citr != citr_end; ++citr) {
          operation_t child_op = *citr;

          auto citr = placed_ops.find(child_op);
          if (citr == placed_ops.end()) {
            throw strip_packing_exception_t("missing placed ops entry.");
          }

          unit_t child_op_end_time =
              (citr->second).y_ + delay(input_dag, child_op);

          curr_op_end_time = std::max(curr_op_end_time, child_op_end_time);
        }

        op_placement_gap.gap_ = (curr_op_end_time  - curr_op_orig_end_time);
        *output = op_placement_gap.gap_;
      }
    }

    // Returns false: if any operations are not original 
    template<typename OutputRectIterator>
    static bool generate_packing_from_feasible_scheduler(dag_t& input_dag,
        size_t resource_upper_bound, OutputRectIterator output)
    {
      typedef mv::lp_scheduler::Feasible_Memory_Schedule_Generator<dag_t>
        scheduler_t;
      typedef op_placement_gap_t op_placement_t;
      typedef typename scheduler_t::scheduled_op_info_t op_info_t;
      typedef typename scheduler_t::traits scheduler_traits_t;


      //STEP-1: first collect the schedule //
      typedef std::vector<op_info_t> schedule_list_t;
      schedule_list_t original_schedule, updated_schedule,
          *final_schedule_ptr = NULL;
      std::vector<rect_t> original_rects;
      size_t first_spill_idx = std::numeric_limits<size_t>::max(); 

      rect_t aux_rect;
      unit_t xmin, ymin, xmax, ymax;
      {
        scheduler_t scheduler_begin(input_dag, resource_upper_bound),
                    scheduler_end;
        size_t curr_idx = 0;
        while (scheduler_begin != scheduler_end) {
          const op_info_t &op_info = *scheduler_begin;
          op_info.print();
          original_schedule.push_back(op_info);

          if (strcmp(op_info.op_type_name(), "ORIGINAL")) {
            first_spill_idx = std::min(first_spill_idx, curr_idx);
          }

          ymin = unit_t(op_info.time_);
          ymax = ymin + delay(input_dag, op_info.op_);
          xmin = xmax = unit_t(0);

          assert(ymin >= 1);
          ymin -= unit_t(1);
          ymax -= unit_t(1);

          if (op_info.has_active_resource()) {
            xmin = unit_t( op_info.begin_resource() );
            xmax = unit_t( op_info.end_resource() );
            assert( xmin >= 1);
            xmin -= unit_t(1);
            //xmax -= unit_t(1);
          } 

          // NOTE: currently we use same delay as the original op for the 
          // spilled write and spilled read.
          rtraits::create_rect(xmin, ymin, xmax, ymax, aux_rect);
          original_rects.push_back( aux_rect );

          ++scheduler_begin; ++curr_idx;
        }
      }

      original_schedule.pop_back();
      final_schedule_ptr = &original_schedule;
      std::unordered_map<operation_t, rect_t> scheduled_ops;

      bool has_spills = (first_spill_idx < original_schedule.size());

      if (has_spills) {
        // rebuild flow and created updated 
        typename scheduler_traits_t::dag_editor_t dag_editor(input_dag);
        typedef typename schedule_list_t::const_iterator sched_iterator_t;

        // Generate DAG edits and the updated schedule //
        Scheduler_DAG_Util::rebuild_data_flow_dag_from_schedule<
          sched_iterator_t, scheduler_traits_t, op_info_t>( dag_editor,
                original_schedule.begin(), original_schedule.end(),
                  std::back_inserter(updated_schedule) );

        // apply edits //
        scheduler_traits_t::apply_edits(dag_editor, input_dag);

        final_schedule_ptr = &updated_schedule;
      } 

      for (size_t i=0; i<final_schedule_ptr->size(); i++) {
        const op_info_t &op_info = final_schedule_ptr->at(i); 
        auto itr = scheduled_ops.find( op_info.op_ );
        if (itr != scheduled_ops.end()) {
          throw strip_packing_exception_t("name conflict in final schedule.");
        }
        scheduled_ops.insert(std::make_pair(op_info.op_, original_rects[i]));
      }

      const_operation_iterator_t node_itr =
        dag_traits::operations_begin(input_dag),
        node_itr_end = dag_traits::operations_end(input_dag);

      for (; node_itr != node_itr_end; ++node_itr) {
        auto itr = scheduled_ops.find( *node_itr );
        if (itr == scheduled_ops.end()) {
          throw strip_packing_exception_t("missing op in the scheduled map");
        }
        *output = itr->second;
        ++output;
      }
      return !has_spills;
    }

    // NOTE: If the schedule has any spills then input_dag will updated with
    // new data flows.
    template<typename OutputIterator>
    static bool derive_all_solution_variables_from_feasible_scheduler(
        dag_t &input_dag, size_t resource_upper_bound,
        OutputIterator output) {

      std::vector<rect_t> feasible_packing;
      generate_packing_from_feasible_scheduler(input_dag, resource_upper_bound,
          std::back_inserter(feasible_packing));

      unit_t z_h = 0, ymin, ymax;
      // STEP-1: now spit out the placement variables first //
      for (auto itr=feasible_packing.begin(); itr!=feasible_packing.end();
            ++itr) {
        const rect_t &r = *itr;
        *output = rtraits::llx(r); ++output;
        *output = rtraits::lly(r); ++output;

        rtraits::yinterval(r, ymin, ymax);
        z_h = std::max(z_h, ymax);
      }

      // STEP-2: now spit out the direction variables //
      for (size_t i=1UL; i<=feasible_packing.size(); i++) {
        for (size_t j=i+1UL; j<=feasible_packing.size(); j++) {
          unit_t derived_vars[4UL];
          derive_direction_variables_from_placement_pair(
              feasible_packing[i-1], feasible_packing[j-1],
              derived_vars[0], derived_vars[1],
              derived_vars[2], derived_vars[3], true);

          // send to output //
          for (size_t i=0; i<4UL; i++) {
            *output = derived_vars[i]; ++output;
          }
        }
      }

      // STEP-3: derive gap variables //
      derive_gap_variables_from_packing(input_dag, feasible_packing.begin(),
          feasible_packing.end(), output);

      *output = z_h; ++output;
    }
    ////////////////////////////////////////////////////////////////////////////


  private:
    void init() {
      constraint_matrix_.clear();
      variable_codec_.reset(N_);
    }

    inline size_t delay(size_t node_id) const { return height(node_id); }
    inline static size_t delay(const dag_t& dag, const operation_t& node) {
      return dag_traits::height(dag, node);
    }
    inline size_t resource_util(size_t node_id) const { return width(node_id); }

    inline unit_t height(size_t node_id) const {
      assert(node_id);
      return heights_[node_id-1UL];
    }
    inline unit_t width(size_t node_id) const {
      assert(node_id);
      return widths_[node_id-1UL];
    }

    ////////////////////////////////////////////////////////////////////////////
    unit_t W_; // strip width //
    unit_t H_; // upper bound on the height //
    size_t N_; // total number of bins //
    std::vector<unit_t> widths_;
    std::vector<unit_t> heights_;
    constraint_matrix_t constraint_matrix_;
    variable_codec_t variable_codec_;
    node_map_t node_map_;
}; // class Strip_Packing_Linear_Program //



} // namespace mv //

#endif
