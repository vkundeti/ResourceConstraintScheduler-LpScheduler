#ifndef TWOD_STRIP_PACKING_GLPK_SOLVER_HPP
#define TWOD_STRIP_PACKING_GLPK_SOLVER_HPP

#include <glpk.h>
#include "2d_strip_packing_lp.hpp"

namespace mv {

struct Strip_Packing_Initial_Feasible_Solution {
  static void set_initial_solution(glp_tree *tree, void *info);
  static double const *init_solution_begin_;
  static double const *init_solution_end_;
}; // struct Strip_Packing_Initial_Feasible_Solution //


template<typename BinTraits, typename DAGTraits>
class Strip_Packing_GLPK_Solver {

  public:

  //////////////////////////////////////////////////////////////////////////////
  typedef BinTraits traits;
  typedef Strip_Packing_Linear_Program<traits, DAGTraits> strip_packing_lp_t;
  typedef typename strip_packing_lp_t::constraint_matrix_t constraint_matrix_t;
  typedef typename strip_packing_lp_t::matrix_entry_t matrix_entry_t;
  typedef typename strip_packing_lp_t::variable_iterator_t variable_iterator_t;
  typedef typename strip_packing_lp_t::unit_t unit_t;
  struct placement_t {
    unit_t x_;
    unit_t y_;
  }; // struct placement_t //
  typedef std::vector<placement_t> strip_packing_solution_t;
  typedef typename strip_packing_solution_t::const_iterator solution_iterator_t;
  //////////////////////////////////////////////////////////////////////////////

  Strip_Packing_GLPK_Solver() : solution_() {}

  template<typename BinIterator>
  solution_iterator_t solution_begin(
      BinIterator input_bins_begin, BinIterator input_bins_end,
      size_t strip_width, size_t strip_height_upper_bound,
      bool dump_solution=false,
      double const *init_solution_begin=NULL,
      double const *init_solution_end=NULL,
      double *final_solution_begin=NULL, double *final_solution_end=NULL) {
    strip_packing_lp_t packing_lp;
    packing_lp.reset(input_bins_begin, input_bins_end,
          strip_width, strip_height_upper_bound);
    packing_lp.generate_constraint_matrix();

    return solution_with_lp_generator(packing_lp, dump_solution,
        init_solution_begin, init_solution_end,
        final_solution_begin, final_solution_end);
  }

  template<typename BinIterator, typename CPairIterator>
  solution_iterator_t solution_begin_with_coinciding_pairs(
      BinIterator input_bins_begin, BinIterator input_bins_end,
      CPairIterator cpair_begin, CPairIterator cpair_end,
      size_t strip_width, size_t strip_height_upper_bound,
      bool dump_solution=false,
      double const *init_solution_begin=NULL,
      double const *init_solution_end=NULL,
      double *final_solution_begin=NULL, double *final_solution_end=NULL) {
    strip_packing_lp_t packing_lp;
    packing_lp.reset(input_bins_begin, input_bins_end,
          strip_width, strip_height_upper_bound);

    if (cpair_begin != cpair_end) {
      packing_lp.add_coinciding_pairs_and_update_variables(cpair_begin,
            cpair_end);
    }
    packing_lp.generate_constraint_matrix();

    return solution_with_lp_generator(packing_lp, dump_solution,
        init_solution_begin, init_solution_end,
        final_solution_begin, final_solution_end);
  }


  solution_iterator_t solve_resource_constraint_scheduling_problem(
      typename DAGTraits::dag_t const & input_dag,
      size_t resource_bound, size_t makespan_upper_bound,
      bool ignore_precedence_constraints=false,
      bool ignore_producer_consumer_constraints=true) {

    strip_packing_lp_t packing_lp;

    packing_lp.reset_from_dag(input_dag, resource_bound, makespan_upper_bound);
    packing_lp.generate_constraint_matrix(input_dag,
          ignore_precedence_constraints, ignore_producer_consumer_constraints);

    return solution_with_lp_generator(packing_lp);
  }


  solution_iterator_t solve_resource_constraint_scheduling_problem(
      typename DAGTraits::dag_t const & input_dag,
      size_t resource_bound, size_t makespan_upper_bound,
      double const *init_solution_begin=NULL,
      double const *init_solution_end=NULL,
      double *final_solution_begin=NULL, double *final_solution_end=NULL,
      bool dump_solution=true) {

    strip_packing_lp_t packing_lp;

    packing_lp.reset_from_dag(input_dag, resource_bound, makespan_upper_bound);
    packing_lp.generate_constraint_matrix(input_dag, false, false);

    return solution_with_lp_generator(packing_lp, dump_solution,
        init_solution_begin, init_solution_end,
        final_solution_begin, final_solution_end);
  }


  solution_iterator_t
    solve_resource_constraint_scheduling_problem_and_get_final_solution(
      typename DAGTraits::dag_t const & input_dag,
      size_t resource_bound, size_t makespan_upper_bound,
      double *final_solution_begin, double *final_solution_end) {

    strip_packing_lp_t packing_lp;

    packing_lp.reset_from_dag(input_dag, resource_bound, makespan_upper_bound);
    packing_lp.generate_constraint_matrix(input_dag, false, false);

    return solution_with_lp_generator(packing_lp, true, NULL, NULL,
          final_solution_begin, final_solution_end);
  }

  template<typename PackingLP>
  solution_iterator_t solution_with_lp_generator(PackingLP const & packing_lp,
      bool dump_solution=false,
      double const *init_sol_var_begin=NULL,
      double const *init_sol_var_end=NULL,
      double *final_solution_begin=NULL, double *final_solution_end=NULL) {

    glp_prob *lp_prob;
    const constraint_matrix_t& matrix = packing_lp.get_constraint_matrix();
    int row_count = (int) matrix.row_dim_ , col_count = (int) matrix.col_dim_;
    bool use_init_solution = (init_sol_var_begin != init_sol_var_end);

    if (!row_count || !col_count) { return solution_.end(); }

    if (use_init_solution) {
      Strip_Packing_Initial_Feasible_Solution::init_solution_begin_
          = init_sol_var_begin;
      Strip_Packing_Initial_Feasible_Solution::init_solution_end_
          = init_sol_var_end;
    }

    // STEP-1: create problem-instance //
    lp_prob = glp_create_prob();
    glp_set_prob_name(lp_prob, "strip-packer");
    glp_set_obj_dir(lp_prob, GLP_MIN);

    glp_add_rows(lp_prob, row_count);

    int width = (int) packing_lp.get_width();
    int height = (int) packing_lp.get_height();
    int curr_row = row_count + 1UL;
    char name_buf[4096UL];
    size_t m = matrix.entries_.size();

    //NOTE: memory allocation needs cleanup //
    int *row_index_array = new int[m+1];
    int *col_index_array = new int[m+1];
    double *value_array = new double[m+1];

    // STEP-2: create rows //
    int eid=1;
    for (const matrix_entry_t& entry : matrix.entries_) {
      if ((entry.index_.row_idx_+1UL) != curr_row) {
        curr_row = entry.index_.row_idx_ + 1UL;
        sprintf(name_buf, "ROW-%d", curr_row);
        glp_set_row_name(lp_prob, curr_row, name_buf);
      }
      int curr_col = entry.index_.col_idx_;
      int curr_value = entry.value_;

      if (curr_col == col_count+1) {
        // set row bounds //
        glp_set_row_bnds(lp_prob, curr_row, GLP_UP, 0.0, double(curr_value));
      } else {
        row_index_array[eid] = curr_row;
        col_index_array[eid] = curr_col;
        value_array[eid] = double(curr_value);
        ++eid;
      }
    }

    // STEP-3: create columns //
    int col_id=1;
    {
      glp_add_cols(lp_prob, col_count);
      for (variable_iterator_t itr=packing_lp.variable_iterator_begin();
            itr!=packing_lp.variable_iterator_end(); ++itr) {
        const char *col_name = *itr;
        glp_set_col_name(lp_prob, col_id, col_name);
        if (col_id != col_count) {
          glp_set_obj_coef(lp_prob, col_id, 0);
        } else {
          glp_set_obj_coef(lp_prob, col_id, double(1));
        }

        if (col_name[0] == 'x') {
          glp_set_col_bnds(lp_prob, col_id, GLP_DB, 0.0, double(width));
        } else if (col_name[0] == 'y') {
          glp_set_col_bnds(lp_prob, col_id, GLP_DB, 0.0, double(height));
        } else if (col_name[0] == 't') {
          glp_set_col_bnds(lp_prob, col_id, GLP_DB, 0.0, double(1.0));
        } else {
          glp_set_col_bnds(lp_prob, col_id, GLP_DB, 0.0, double(height));
        }
        glp_set_col_kind(lp_prob, col_id, GLP_IV);
        ++col_id;
      }
    }

    glp_load_matrix(lp_prob, eid-1, row_index_array, col_index_array,
          value_array);

    glp_iocp param;

    glp_write_lp(lp_prob, NULL, "packing_lp.cplex");
    {
      glp_init_iocp(&param);
      param.presolve= GLP_OFF;
      glp_simplex(lp_prob, NULL);
      if (use_init_solution) {
        param.cb_func =
            Strip_Packing_Initial_Feasible_Solution::set_initial_solution;
      }
      int err = glp_intopt(lp_prob, &param);
      if (err || (glp_mip_status(lp_prob) == GLP_NOFEAS) ) {
        return solution_.end();
      }
    }

    solution_.clear();
    col_id = 1;
    size_t dim_variables = 2UL*(packing_lp.dimension());
    while (col_id <= dim_variables) {
      placement_t pl;
      pl.x_ = glp_mip_col_val(lp_prob, col_id++);
      pl.y_ = glp_mip_col_val(lp_prob, col_id++);
      solution_.push_back(pl);
    }

    if ((dump_solution) || (final_solution_begin != final_solution_end) ) {
      //printf("===============================\n");
      col_id = 1;
      for (variable_iterator_t itr=packing_lp.variable_iterator_begin();
            itr!=packing_lp.variable_iterator_end(); ++itr) {
        const char *col_name = *itr;

        if (final_solution_begin != final_solution_end) {
          final_solution_begin[col_id] = glp_mip_col_val(lp_prob, col_id);
        }

        if (dump_solution) {
          printf("%s = %f\n", *itr, glp_mip_col_val(lp_prob, col_id));
        }

        col_id++;
      }
      //printf("===============================\n");
    }

    if (row_index_array) { 
      delete [] row_index_array;
      row_index_array = NULL;
    }

    if (col_index_array) { 
      delete [] col_index_array;
      col_index_array = NULL;
    }

    if (value_array) {
      delete [] value_array;
      value_array = NULL;
    }

    return solution_.begin();
  }


  solution_iterator_t solution_end() { return solution_.end(); }
  void reset() { solution_.clear(); }


  private:


  strip_packing_solution_t solution_;
}; // class Strip_Packing_GLPK_Solver //

} // namespace mv //

#endif
